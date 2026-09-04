#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboCup医疗配送比赛总控制器（负责导航、定位监控和比赛流程编排）。

完整流程：
    等待导航依赖和可靠定位
      -> 等待比赛开始信号
      -> 前往护士站读取任务码
      -> 按任务码依次访问两个病床
      -> 在病床发布药箱任务上下文并等待外部节点完成
      -> 返回起点并保持静止至少5秒

本节点不直接实现二维码识别、机械臂动作和药品处理，而是通过
``/competition/task_code``、``/competition/task_context`` 和
``/competition/station_done`` 等小型ROS接口与外部功能解耦。这样后续接入
扫码器或机械臂时，不需要改动本文件中的比赛主流程。

安全设计要点：
    * 定位数据过期、定位降级超时或紧急停止时立即停止底盘；
    * 比赛剩余时间达到返航预留值时放弃当前访问并返航；
    * 到点必须同时满足“完成信号”和“位置误差在容差内”；
    * 每次切换导航目标前先禁用PID跟随器并连续发布零速度。
"""

import json
import math
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Empty

try:
    # 自定义PID路径跟随器服务。开发机未编译roborts_msgs时仍允许加载本文件，
    # 是否必须存在由~require_pid_service参数决定。
    from roborts_msgs.srv import PidPlannerStatus, PidPlannerStatusRequest
except ImportError:
    PidPlannerStatus = None
    PidPlannerStatusRequest = None


# =============================================================================
# 任务码和通用数学工具
# =============================================================================

# 二位任务码的每一位表示对应访问使用的药箱编号。
# value = ((第一次访问的病床, 药箱), (第二次访问的病床, 药箱))
VALID_TASKS = {
    11: ((1, 1), (3, 3)),
    13: ((1, 3), (3, 1)),
    31: ((3, 1), (1, 3)),
    33: ((3, 3), (1, 1)),
}


def decode_task_code(code):
    """校验任务码，并返回两次访问的 ``(病床编号, 药箱编号)``。"""
    value = int(code)
    if value not in VALID_TASKS:
        raise ValueError("task code must be one of 11, 13, 31, 33")
    return VALID_TASKS[value]


def yaw_to_quaternion(yaw):
    """将平面yaw角转换成四元数中需要的z、w两个分量。"""
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def quaternion_to_yaw(quaternion):
    """不依赖tf，直接从geometry_msgs/Quaternion中提取平面yaw角。"""
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """把任意弧度角归一化到[-pi, pi]，用于计算最短角度误差。"""
    return math.atan2(math.sin(angle), math.cos(angle))


# =============================================================================
# 比赛总控制器
# =============================================================================
class CompetitionMaster:
    """集中管理比赛状态、导航、定位安全检查及外部站点任务接口。"""

    def __init__(self):
        # 回调线程与主流程会同时访问定位、任务码等共享数据，统一用可重入锁保护。
        self.lock = threading.RLock()

        # Event用于把ROS异步回调转换成主流程可等待的同步事件。
        self.start_event = threading.Event()
        self.emergency_event = threading.Event()
        self.station_done_event = threading.Event()

        # ---------------------------------------------------------------------
        # 比赛策略参数
        # ---------------------------------------------------------------------
        self.frame_id = rospy.get_param("~frame_id", "map")
        # 无新鲜二维码时使用的测试兜底任务码，只允许11/13/31/33。
        self.default_task_code = int(rospy.get_param("~default_task_code", 11))
        decode_task_code(self.default_task_code)
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        # True：病床处只停留固定时间；False：等待外部节点发布station_done。
        self.skip_station_actions = bool(rospy.get_param("~skip_station_actions", True))
        self.station_hold_seconds = float(rospy.get_param("~station_hold_seconds", 2.0))
        self.station_action_timeout = float(rospy.get_param("~station_action_timeout", 30.0))
        # 到护士站后等待新任务码的最长时间；0表示立即使用当前值或兜底值。
        self.task_code_wait_seconds = float(rospy.get_param("~task_code_wait_seconds", 0.0))
        self.require_fresh_task_code = bool(rospy.get_param("~require_fresh_task_code", True))
        self.competition_timeout = float(rospy.get_param("~competition_timeout", 180.0))
        # 剩余时间低于该值时不再前往病床，优先保证能够返航。
        self.reserve_home_seconds = float(rospy.get_param("~reserve_home_seconds", 35.0))
        # 规则要求回到起点后停车超过5秒，因此强制下限为5.1秒。
        self.home_stop_seconds = max(5.1, float(rospy.get_param("~home_stop_seconds", 5.5)))
        self.goal_retries = int(rospy.get_param("~goal_retries", 1))

        # ---------------------------------------------------------------------
        # 定位、导航依赖及到点判据参数
        # ---------------------------------------------------------------------
        self.localization_freshness = float(rospy.get_param("~localization_freshness", 2.0))
        self.initial_localization_timeout = float(rospy.get_param("~initial_localization_timeout", 20.0))
        self.max_fallback_seconds = float(rospy.get_param("~max_fallback_seconds", 10.0))
        self.relocalization_wait_seconds = float(rospy.get_param("~relocalization_wait_seconds", 8.0))
        self.dependency_timeout = float(rospy.get_param("~dependency_timeout", 20.0))
        self.goal_settle_seconds = max(0.0, float(rospy.get_param("~goal_settle_seconds", 0.6)))
        self.stop_publish_cycles = max(3, int(rospy.get_param("~stop_publish_cycles", 5)))
        self.require_pid_service = bool(rospy.get_param("~require_pid_service", True))
        # 调试时可允许只凭距离判定到点；正式比赛建议保持False。
        self.allow_distance_only_arrival = bool(rospy.get_param("~allow_distance_only_arrival", False))

        # ---------------------------------------------------------------------
        # ROS接口名称和由launch/YAML提供的目标点配置
        # ---------------------------------------------------------------------
        self.pose_topic = rospy.get_param("~pose_topic", "/unified_pose")
        self.localization_status_topic = rospy.get_param("~localization_status_topic", "/localization_status")
        self.target_done_topic = rospy.get_param("~target_done_topic", "/target_done")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.pid_status_service = rospy.get_param("~pid_status_service", "/pid_planner_status")
        self.clear_costmaps_service = rospy.get_param("~clear_costmaps_service", "/move_base/clear_costmaps")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.waypoints = rospy.get_param("~waypoints", {})
        self._validate_waypoints()

        # ---------------------------------------------------------------------
        # 回调更新的实时状态；wall_time使用monotonic避免系统时间跳变
        # ---------------------------------------------------------------------
        self.pose = None
        self.pose_wall_time = 0.0
        self.localization_ok = False
        self.localization_status_seen = False
        self.task_code = None
        self.task_code_wall_time = 0.0
        self.task_code_source = "unset"
        self.target_done = 0
        self.target_done_events = 0
        self.target_done_wall_time = 0.0
        self.match_started_at = None
        self.fallback_started_at = None

        # ---------------------------------------------------------------------
        # 对外状态：供显示、扫码、机械臂及裁判辅助节点订阅
        # ---------------------------------------------------------------------
        self.state_pub = rospy.Publisher("/competition/state", String, queue_size=10, latch=True)
        self.station_pub = rospy.Publisher("/competition/current_station", String, queue_size=10, latch=True)
        self.context_pub = rospy.Publisher("/competition/task_context", String, queue_size=10, latch=True)
        self.finished_pub = rospy.Publisher("/competition/finished", Bool, queue_size=1, latch=True)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        # 控制输入和传感器状态订阅。
        rospy.Subscriber("/competition/start", Bool, self._start_cb, queue_size=1)
        rospy.Subscriber("/competition/emergency_stop", Bool, self._emergency_cb, queue_size=1)
        rospy.Subscriber("/competition/task_code", Int32, self._task_code_cb, queue_size=1)
        rospy.Subscriber("/competition/station_done", Bool, self._station_done_cb, queue_size=1)
        rospy.Subscriber(self.localization_status_topic, Bool, self._localization_cb, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self._pose_cb, queue_size=20)
        rospy.Subscriber(self.target_done_topic, Int32, self._target_done_cb, queue_size=20)

        # move_base负责生成全局路径，自定义PID服务负责启停实际路径跟随。
        self.move_base = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)
        self.clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_service, Empty)
        self.pid_status = None
        if PidPlannerStatus is not None:
            self.pid_status = rospy.ServiceProxy(self.pid_status_service, PidPlannerStatus)

        self._publish_state("BOOT")
        self.finished_pub.publish(False)
        rospy.on_shutdown(self.stop_motion)

    def _validate_waypoints(self):
        """启动前检查四个必需目标点及x/y/yaw字段，尽早暴露配置错误。"""
        for name in ("nurse", "bed1", "bed3", "home"):
            if name not in self.waypoints:
                raise rospy.ROSInitException("missing waypoint: %s" % name)
            for key in ("x", "y", "yaw"):
                if key not in self.waypoints[name]:
                    raise rospy.ROSInitException("waypoint %s missing %s" % (name, key))

    def _start_cb(self, msg):
        """接收一次True后永久置位本场比赛的开始事件。"""
        if msg.data:
            self.start_event.set()

    def _emergency_cb(self, msg):
        """紧急停止一旦触发便锁存，并立即停止所有运动源。"""
        if msg.data:
            self.emergency_event.set()
            self.stop_motion()

    def _station_done_cb(self, msg):
        """接收病床外部动作节点的完成确认。"""
        if msg.data:
            self.station_done_event.set()

    def _task_code_cb(self, msg):
        """只接受合法任务码，并记录接收时间以支持“必须是新码”的规则。"""
        try:
            decode_task_code(msg.data)
        except ValueError:
            rospy.logwarn("Ignoring invalid competition task code: %s", msg.data)
            return
        with self.lock:
            self.task_code = int(msg.data)
            self.task_code_wall_time = time.monotonic()
        rospy.loginfo("Accepted competition task code: %d", msg.data)

    def _localization_cb(self, msg):
        """记录融合定位健康状态，并计时连续降级持续时间。"""
        with self.lock:
            self.localization_status_seen = True
            self.localization_ok = bool(msg.data)
            if self.localization_ok:
                self.fallback_started_at = None
            elif self.fallback_started_at is None:
                self.fallback_started_at = time.monotonic()

    def _pose_cb(self, msg):
        """保存最新统一位姿及其本机接收时间。"""
        with self.lock:
            self.pose = msg
            self.pose_wall_time = time.monotonic()

    def _target_done_cb(self, msg):
        """记录自定义路径跟随器发布的到点事件。"""
        with self.lock:
            value = int(msg.data)
            # 跟随器只在到点时发布。除保存其累计值外，另行统计消息事件次数，
            # 防止跟随器重启后累计值归零，导致总控永远等不到“数值继续递增”。
            self.target_done = value
            self.target_done_events += 1
            self.target_done_wall_time = time.monotonic()

    def _publish_state(self, state):
        """发布并打印当前比赛状态；话题带latch，后启动节点也能获得最新值。"""
        self.state_pub.publish(String(data=state))
        rospy.loginfo("competition state -> %s", state)

    def _publish_context(self, **kwargs):
        """以JSON发布结构化任务上下文，供扫码/机械臂/记录节点解耦使用。"""
        payload = dict(kwargs)
        payload["stamp"] = rospy.Time.now().to_sec()
        self.context_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _abort(self, state, message):
        """进入安全终止状态，停止全部运动源并发布未完成结果。"""
        rospy.logerr("%s", message)
        self._publish_state(state)
        self.stop_motion()
        self.finished_pub.publish(False)

    def _wait_for_dependencies(self):
        """等待move_base及可选的PID服务就绪，避免比赛开始后才发现依赖缺失。"""
        self._publish_state("WAIT_DEPENDENCIES")
        if not self.move_base.wait_for_server(rospy.Duration(self.dependency_timeout)):
            self._abort("ABORT_NO_MOVE_BASE", "move_base action server was not available")
            return False

        if self.require_pid_service:
            if self.pid_status is None:
                self._abort("ABORT_NO_PID_SERVICE_TYPE", "roborts_msgs/PidPlannerStatus could not be imported")
                return False
            try:
                rospy.wait_for_service(self.pid_status_service, timeout=self.dependency_timeout)
            except rospy.ROSException:
                self._abort("ABORT_NO_PID_SERVICE", "PID follower service was not available")
                return False
        return True

    def _pose_is_fresh(self):
        """判断统一位姿是否存在且未超过允许的数据陈旧时间。"""
        with self.lock:
            return self.pose is not None and (time.monotonic() - self.pose_wall_time) <= self.localization_freshness

    def _initial_localization_ready(self):
        """初始定位必须同时满足：收到健康状态、状态正常、位姿新鲜。"""
        with self.lock:
            return self.localization_status_seen and self.localization_ok and self._pose_is_fresh()

    def _fallback_too_long(self):
        """定位连续处于降级/不可靠状态是否超过最大容忍时间。"""
        with self.lock:
            if self.localization_ok or self.fallback_started_at is None:
                return False
            return (time.monotonic() - self.fallback_started_at) > self.max_fallback_seconds

    def _remaining_time(self):
        """返回本场比赛剩余秒数；比赛未开始时返回完整时限。"""
        if self.match_started_at is None:
            return self.competition_timeout
        return self.competition_timeout - (time.monotonic() - self.match_started_at)

    def _wait_for_initial_localization(self):
        """在启动超时内等待首个可靠定位，期间持续响应紧急停止。"""
        self._publish_state("WAIT_LOCALIZATION")
        deadline = time.monotonic() + self.initial_localization_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.emergency_event.is_set():
                return False
            if self._initial_localization_ready():
                return True
            rate.sleep()
        return False

    def _set_pid_state(self, state):
        """设置自定义PID路径跟随器状态；0为停用，2为开始跟随路径。"""
        if self.pid_status is None:
            return not self.require_pid_service
        try:
            request = PidPlannerStatusRequest()
            request.planner_state = int(state)
            request.max_x_speed = -1.0
            request.max_y_speed = -1.0
            request.yaw_speed = -1.0
            response = self.pid_status(request)
            return int(response.result) == 1
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(5.0, "PID planner service unavailable: %s", exc)
            return False

    def _publish_zero(self, cycles=None):
        """连续发布多帧零速度，降低单帧丢失或其他速度源残留的风险。"""
        zero = Twist()
        count = self.stop_publish_cycles if cycles is None else max(1, int(cycles))
        for _ in range(count):
            self.cmd_vel_pub.publish(zero)
            rospy.sleep(0.02)

    def stop_motion(self):
        """统一安全停车：取消move_base目标、停用PID跟随器并发布零速度。"""
        try:
            self.move_base.cancel_all_goals()
        except Exception:
            pass
        self._set_pid_state(0)
        self._publish_zero()

    def _distance_to(self, waypoint):
        """计算当前统一位姿到目标点的二维欧氏距离。"""
        with self.lock:
            if self.pose is None:
                return float("inf")
            p = self.pose.pose.pose.position
            return math.hypot(p.x - float(waypoint["x"]), p.y - float(waypoint["y"]))

    def _yaw_error_to(self, waypoint):
        """计算当前朝向与目标yaw之间的最短绝对角度误差。"""
        with self.lock:
            if self.pose is None:
                return float("inf")
            current_yaw = quaternion_to_yaw(self.pose.pose.pose.orientation)
            return abs(normalize_angle(float(waypoint["yaw"]) - current_yaw))

    def _build_goal(self, waypoint):
        """把参数字典中的x/y/yaw转换为move_base目标消息。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(waypoint["x"])
        goal.target_pose.pose.position.y = float(waypoint["y"])
        qz, qw = yaw_to_quaternion(float(waypoint["yaw"]))
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        return goal

    def _wait_for_relocalization(self):
        """停车等待定位恢复；超时或急停均返回False。"""
        self.stop_motion()
        self._publish_state("WAIT_RELOCALIZATION")
        deadline = time.monotonic() + self.relocalization_wait_seconds
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.emergency_event.is_set():
                return False
            with self.lock:
                ready = self.localization_ok and self._pose_is_fresh()
            if ready:
                return True
            rate.sleep()
        return False

    def navigate(self, waypoint_name, allow_reserve_return=True):
        """
        导航到指定目标点，并返回字符串结果供主流程决定下一步。

        到点判据默认要求：move_base成功或收到本次目标后的target_done事件，
        并且当前位置距离目标不超过tolerance。仅在显式启用
        allow_distance_only_arrival时，距离本身也可以作为完成来源。
        """
        waypoint = self.waypoints[waypoint_name]
        tolerance = float(waypoint.get("tolerance", 0.30))
        timeout = float(waypoint.get("timeout", 50.0))

        for attempt in range(self.goal_retries + 1):
            # 每次尝试前先处理最高优先级的急停和返航时间预留。
            if self.emergency_event.is_set():
                return "emergency"
            if allow_reserve_return and self._remaining_time() <= self.reserve_home_seconds:
                rospy.logwarn("Home-time reserve reached; abandoning %s", waypoint_name)
                return "reserve"

            self._publish_state("NAV_%s" % waypoint_name.upper())
            self.station_pub.publish(String(data=waypoint_name))
            # 替换move_base目标前先停用跟随器，防止新路径生成的短暂间隙内
            # 机器人继续沿上一个目标的旧路径运动。
            self.stop_motion()
            with self.lock:
                # 基准值用于确认后续target_done事件确实属于这次导航。
                target_event_baseline = self.target_done_events
            goal_sent_at = time.monotonic()
            self.move_base.send_goal(self._build_goal(waypoint))

            settle_deadline = time.monotonic() + self.goal_settle_seconds
            # 给move_base留出生成新全局路径的时间，此期间持续压零速度。
            while not rospy.is_shutdown() and time.monotonic() < settle_deadline:
                if self.emergency_event.is_set() or not self._pose_is_fresh():
                    self.stop_motion()
                    return "emergency" if self.emergency_event.is_set() else "localization_lost"
                self._publish_zero(cycles=1)

            if not self._set_pid_state(2):
                self.stop_motion()
                rospy.logerr("Could not enable PID path follower")
                return "pid_unavailable"

            deadline = time.monotonic() + timeout
            rate = rospy.Rate(10)

            while not rospy.is_shutdown() and time.monotonic() < deadline:
                # 循环内按优先级检查：急停、总超时、返航预留、定位状态、到点。
                if self.emergency_event.is_set():
                    self.stop_motion()
                    return "emergency"
                if self._remaining_time() <= 0.0:
                    self.stop_motion()
                    return "match_timeout"
                if allow_reserve_return and self._remaining_time() <= self.reserve_home_seconds:
                    self.stop_motion()
                    return "reserve"
                if not self._pose_is_fresh():
                    rospy.logwarn("Unified pose became stale; stopping for relocalization")
                    if not self._wait_for_relocalization():
                        return "localization_lost"
                    break
                if self._fallback_too_long():
                    rospy.logwarn("Localization fallback exceeded %.1f s", self.max_fallback_seconds)
                    if not self._wait_for_relocalization():
                        return "localization_lost"
                    break

                distance = self._distance_to(waypoint)
                with self.lock:
                    # 同时检查事件序号和时间，排除上一次目标遗留的完成消息。
                    custom_arrived = (
                        self.target_done_events > target_event_baseline
                        and self.target_done_wall_time >= goal_sent_at
                    )
                action_state = self.move_base.get_state()
                standard_arrived = action_state == GoalStatus.SUCCEEDED
                distance_arrived = self.allow_distance_only_arrival and distance <= tolerance
                # 即使收到完成信号，也要用实时位姿二次确认距离，避免误判。
                if (custom_arrived or standard_arrived or distance_arrived) and distance <= tolerance:
                    self.stop_motion()
                    yaw_error = self._yaw_error_to(waypoint)
                    self._publish_context(
                        event="navigation_arrived",
                        station=waypoint_name,
                        distance_error=distance,
                        yaw_error=yaw_error,
                        arrival_source=(
                            "target_done" if custom_arrived else
                            "move_base" if standard_arrived else
                            "distance_only"
                        ),
                    )
                    rospy.loginfo(
                        "Arrived at %s (position error %.3f m, yaw error %.3f rad)",
                        waypoint_name, distance, yaw_error)
                    return "arrived"
                if action_state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
                    rospy.logwarn("move_base ended with state %d at %s", action_state, waypoint_name)
                    break
                rate.sleep()

            self.stop_motion()
            self._publish_context(
                event="navigation_attempt_failed",
                station=waypoint_name,
                attempt=attempt + 1,
                action_state=self.move_base.get_state(),
                distance_error=self._distance_to(waypoint),
            )
            if attempt < self.goal_retries:
                # 重试前清理代价地图，用于恢复临时障碍或规划器异常。
                rospy.logwarn("Retrying %s (%d/%d)", waypoint_name, attempt + 1, self.goal_retries)
                try:
                    self.clear_costmaps()
                except (rospy.ServiceException, rospy.ROSException):
                    pass
                rospy.sleep(1.0)
        return "failed"

    def _get_task_code(self, not_before=0.0):
        """
        等待合法任务码；require_fresh_task_code为True时，只接受到达护士站后
        新收到的码。超时后使用default_task_code，方便无扫码器时联调。
        """
        deadline = time.monotonic() + self.task_code_wait_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                fresh_enough = (
                    not self.require_fresh_task_code
                    or self.task_code_wall_time >= not_before
                )
                if self.task_code in VALID_TASKS and fresh_enough:
                    self.task_code_source = "qr"
                    return self.task_code
            if self.emergency_event.is_set():
                return None
            rospy.sleep(0.05)
        with self.lock:
            fresh_enough = (
                not self.require_fresh_task_code
                or self.task_code_wall_time >= not_before
            )
            if self.task_code in VALID_TASKS and fresh_enough:
                self.task_code_source = "qr"
                return self.task_code
        rospy.logwarn("No fresh QR task code received; using test fallback code %d", self.default_task_code)
        self.task_code_source = "default_fallback"
        return self.default_task_code

    def station_placeholder(self, visit_index, bed_id, box_id, task_code):
        """
        病床动作接口占位层。

        skip_station_actions=True时仅停车station_hold_seconds；正式接入机械臂后
        设为False，由外部节点读取task_context并在完成后发布station_done。
        """
        self.stop_motion()
        self._publish_state("BED%d_ACTION_PLACEHOLDER" % bed_id)
        self.station_pub.publish(String(data="bed%d" % bed_id))
        # 必须先清事件再发布请求；否则外部节点若回复很快，可能恰好在publish()
        # 与clear()之间完成，导致总控把刚收到的完成事件误清除。
        self.station_done_event.clear()
        self._publish_context(
            task_code=task_code,
            visit_index=visit_index,
            bed_id=bed_id,
            medicine_box=box_id,
            requires_station_action=not self.skip_station_actions,
        )
        deadline = time.monotonic() + (
            self.station_hold_seconds if self.skip_station_actions else self.station_action_timeout
        )
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            # 病床动作期间也持续监控急停和返航预留，并重复发布零速度。
            if self.emergency_event.is_set():
                return False
            if not self.skip_station_actions and self.station_done_event.is_set():
                return True
            if self._remaining_time() <= self.reserve_home_seconds:
                return False
            self.cmd_vel_pub.publish(Twist())
            rate.sleep()
        return self.skip_station_actions

    def _hold_at_home(self):
        """返航到点后持续发布零速度，确保满足比赛规定的停车时间。"""
        self._publish_state("HOLD_HOME_STOP")
        self.stop_motion()
        deadline = time.monotonic() + self.home_stop_seconds
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.emergency_event.is_set():
                self.stop_motion()
                return False
            self._publish_zero(cycles=1)
            rate.sleep()
        return not rospy.is_shutdown()

    def _return_home(self):
        """返航不再受reserve_home_seconds限制，到点后执行停车保持。"""
        result = self.navigate("home", allow_reserve_return=False)
        if result == "arrived":
            return self._hold_at_home()
        return False

    def run(self):
        """执行一场完整比赛，是本控制器唯一的正式主流程入口。"""
        # 1. 启动门槛：依赖与定位均正常后才允许进入等待开始状态。
        rospy.loginfo("Waiting for navigation dependencies")
        if not self._wait_for_dependencies():
            return
        if not self._wait_for_initial_localization():
            self._abort("ABORT_NO_LOCALIZATION", "Reliable initial localization was not obtained")
            return

        self._publish_state("WAIT_START")
        # 2. 等待裁判/上位机开始信号；auto_start仅用于独立调试。
        if self.auto_start:
            self.start_event.set()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.start_event.is_set():
            if self.emergency_event.is_set():
                self._publish_state("EMERGENCY_STOP")
                return
            rate.sleep()

        self.match_started_at = time.monotonic()
        self._publish_state("RUNNING")

        # 3. 首先到护士站。失败时不再执行送药，但仍尽力安全返航。
        nurse_result = self.navigate("nurse")
        if nurse_result != "arrived":
            self._publish_state("NURSE_FAILED_RETURN_HOME")
            if self._return_home():
                self._publish_state("ABORT_NURSE_FAILED_RETURNED_HOME")
            else:
                self._abort("ABORT_NURSE_AND_HOME_FAILED", "Nurse navigation and return-home both failed")
            return

        self.stop_motion()
        task_scan_started_at = time.monotonic()
        # 4. 请求任务二维码；任务码决定两个病床的顺序及对应药箱。
        self._publish_state("WAIT_TASK_ORDER")
        self._publish_context(event="request_task_scan", valid_codes=sorted(VALID_TASKS.keys()))
        task_code = self._get_task_code(not_before=task_scan_started_at)
        if task_code is None:
            self.stop_motion()
            self._publish_state("EMERGENCY_STOP")
            return
        visits = decode_task_code(task_code)
        self._publish_context(task_code=task_code, visits=visits, source=self.task_code_source)

        all_visits_completed = True
        # 5. 依次导航至两个病床，并在每个病床等待对应外部动作完成。
        for visit_index, (bed_id, box_id) in enumerate(visits, start=1):
            result = self.navigate("bed%d" % bed_id)
            if result != "arrived":
                rospy.logwarn("Visit %d stopped (%s); returning home", visit_index, result)
                all_visits_completed = False
                break
            if not self.station_placeholder(visit_index, bed_id, box_id, task_code):
                rospy.logwarn("Station phase ended early; returning home")
                all_visits_completed = False
                break

        if self.emergency_event.is_set():
            self.stop_motion()
            self._publish_state("EMERGENCY_STOP")
            return

        # 6. 无论病床任务是否全部完成，只要没有急停都尝试返回起点。
        if self._return_home():
            if all_visits_completed:
                self._publish_state("COMPLETED")
                self.finished_pub.publish(True)
            else:
                self._publish_state("PARTIAL_RETURNED_HOME")
                self.finished_pub.publish(False)
        else:
            self.stop_motion()
            self._publish_state("ABORT_HOME_FAILED")


# =============================================================================
# ROS节点入口
# =============================================================================
if __name__ == "__main__":
    rospy.init_node("medical_competition_master")
    controller = None
    try:
        controller = CompetitionMaster()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:
        rospy.logfatal("Competition master crashed: %s", exc)
        if controller is not None:
            controller.stop_motion()
        raise
