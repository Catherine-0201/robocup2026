import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.linear_model import RANSACRegressor
import math
from std_msgs.msg import Float32

class ScanToLineFitter:
    def __init__(self):
        self.laser_projector = lg.LaserProjection()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.yaw_pub = rospy.Publisher('/lidar_yaw', Float32, queue_size=1)

    def scan_callback(self, scan):
        # 步骤1: 将LaserScan转换为PointCloud2
        cloud = self.laser_projector.projectLaser(scan)
        
        # 步骤2: 提取前方点云数据（角度范围±60°，距离1.5-2.5m）
        points = []
        for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[0], point[1], point[2]
            dist = math.sqrt(x**2 + y**2)
            angle = math.atan2(y, x)
            if abs(angle) < math.radians(60) and 0.3 < dist < 0.7:
                points.append([x, y])
        
        # 如果点数不足，跳过或警告
        if len(points) < 10:
            rospy.logwarn("Not enough points")
            return
        
        # 转换为numpy数组
        points = np.array(points)
        
        # 步骤3: 使用RANSAC拟合直线（y = mx + b）
        X = points[:, 0].reshape(-1, 1)
        y = points[:, 1]
        model = RANSACRegressor()
        model.fit(X, y)
        m = model.estimator_.coef_[0]
        b = model.estimator_.intercept_
        
        # 步骤4: 计算法向量（对于y=mx+b，法向量正比于(m, -1)）
        normal = np.array([m, -1])
        normal = normal / np.linalg.norm(normal)  # 归一化成单位向量
        
        # 步骤5: 确定方向 - 不指向原点
        # 计算原点到直线上的一个点的向量，检查与法向量的点积
        # 如果点积 < 0，法向量指向原点，反转使其指向远离原点
        test_point = [points[0][0], points[0][1]]  # 取一个拟合点
        vec_to_point = np.array(test_point)
        dot = np.dot(normal, vec_to_point)
        if dot < 0:
            normal = -normal  # 反转方向，使其不指向原点（即指向远离原点）
        
        # 输出法向量
        rospy.loginfo("Unit normal vector (away from origin): {}".format(normal))
        
        # 计算法向量与x轴的夹角（雷达坐标系：x朝前，y朝左）
        # 使用atan2计算角度，朝左角度为正，朝右角度为负
        angle_rad = math.atan2(normal[1], normal[0])
        angle_deg = math.degrees(angle_rad)
        
        # 输出角度信息
        rospy.loginfo("Normal vector angle with x-axis: {:.2f} degrees".format(angle_deg))
        # rospy.loginfo("Normal vector angle with x-axis: {:.4f} radians".format(angle_rad))
        
        # 发布yaw角度数据
        yaw_msg = Float32()
        yaw_msg.data = angle_deg
        self.yaw_pub.publish(yaw_msg)

if __name__ == '__main__':
    rospy.init_node('scan_to_line_fitter')
    fitter = ScanToLineFitter()
    rospy.spin()
