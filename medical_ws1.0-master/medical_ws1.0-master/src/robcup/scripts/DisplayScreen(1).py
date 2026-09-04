#!/usr/bin/env python3
import sys
import rospy
import csv
import yaml
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QScrollArea, QDesktopWidget, QMessageBox
from PyQt5.QtGui import QIcon
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import os
from datetime import datetime
import numpy as np
import rospkg
class LaserDisplay(QWidget):

    update_distances_signal = pyqtSignal(float, float, float)
    update_code_signal = pyqtSignal(str, str, str)
    update_position_signal = pyqtSignal(float, float)
    update_yaw_signal = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        
        # 设置窗口标题
        self.setWindowTitle('机器人实时信息显示界面')
        
        # 设置窗口尺寸
        screen = QDesktopWidget().availableGeometry()  
        self.setGeometry(0, 0, int(0.8 * screen.width()), int(0.5 * screen.height()))  

        # 使用rospkg获取包路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robcup')  # 替换为你的包名
        # 存储数据至data文件夹中
        if not os.path.exists(package_path+'/data'):
            os.makedirs(package_path+'/data')
        with open(package_path + '/data/distance_data.csv', 'w') as f:
            f.truncate()
        with open(package_path + '/data/yaw_data.csv', 'w') as f:
            f.truncate()
        with open(package_path + '/data/coordinate_data.csv', 'w') as f:
            f.truncate()

        self.qrcode = "尚未可知"
        self.first_code = "尚未可知"
        self.third_code = "尚未可知"
        
        # 初始化坐标信息
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # 初始化yaw角度信息
        self.robot_yaw = 0.0

        self.main_layout = QVBoxLayout()
        label_style = """
            QLabel {
                border: 3px solid black;
                padding: 12px;
                font-size: 20px;
                font-weight: bold;
            }
        """
        # 条形码信息显示
        self.barcode_layout = QHBoxLayout()
        self.qrcode_label = QLabel('二维码：'+self.qrcode)
        self.first_layout = QHBoxLayout()
        self.third_layout = QHBoxLayout()
        self.first_label = QLabel('1号病床: '+self.first_code)
        self.third_label = QLabel('3号病床: '+self.third_code)

        self.qrcode_label.setStyleSheet(label_style)
        self.first_label.setStyleSheet(label_style)
        self.third_label.setStyleSheet(label_style)

        self.barcode_layout.addWidget(self.qrcode_label)
        self.first_layout.addWidget(self.first_label)
        self.third_layout.addWidget(self.third_label)

        self.main_layout.addLayout(self.barcode_layout)
        self.main_layout.addLayout(self.first_layout)
        self.main_layout.addLayout(self.third_layout)

        # 机器人坐标信息显示
        self.position_layout = QHBoxLayout()
        self.position_label = QLabel('机器人坐标信息：')
        self.x_label = QLabel('X坐标: N/A')
        self.y_label = QLabel('Y坐标: N/A')

        self.position_label.setStyleSheet(label_style)
        self.x_label.setStyleSheet(label_style)
        self.y_label.setStyleSheet(label_style)

        self.position_layout.addWidget(self.position_label)
        self.position_layout.addWidget(self.x_label)
        self.position_layout.addWidget(self.y_label)
        self.main_layout.addLayout(self.position_layout)

        # 机器人yaw角度信息显示
        self.yaw_layout = QHBoxLayout()
        self.yaw_label = QLabel('机器人Yaw角度：')
        self.yaw_value_label = QLabel('Yaw角度: N/A')

        self.yaw_label.setStyleSheet(label_style)
        self.yaw_value_label.setStyleSheet(label_style)

        self.yaw_layout.addWidget(self.yaw_label)
        self.yaw_layout.addWidget(self.yaw_value_label)
        self.main_layout.addLayout(self.yaw_layout)


        # 雷达距离信息显示
        self.horizontal_layout = QHBoxLayout()
        self.lidar_distance = QLabel('雷达测距信息：')
        self.front_label = QLabel('前方: N/A')
        self.left_label = QLabel('左侧: N/A')
        self.right_label = QLabel('右侧: N/A')


        self.front_label.setStyleSheet(label_style)
        self.left_label.setStyleSheet(label_style)
        self.right_label.setStyleSheet(label_style)

        self.horizontal_layout.addWidget(self.lidar_distance)
        self.horizontal_layout.addWidget(self.front_label)
        self.horizontal_layout.addWidget(self.left_label)
        self.horizontal_layout.addWidget(self.right_label)
        self.main_layout.addLayout(self.horizontal_layout)

        # 操作按钮
        self.button_layout = QHBoxLayout()

        self.save_label = QLabel('保存定位信息：')
        self.button_layout.addWidget(self.save_label)
        self.save_button = QPushButton('保存当前定位')
        self.save_button.clicked.connect(self.save_distances_to_file)
        self.button_layout.addWidget(self.save_button)

        self.save_yaw_button = QPushButton('保存yaw角度')
        self.save_yaw_button.clicked.connect(self.save_yaw_to_file)
        self.button_layout.addWidget(self.save_yaw_button)

        self.save_coordinate_button = QPushButton('保存坐标')
        self.save_coordinate_button.clicked.connect(self.save_coordinate_to_file)
        self.button_layout.addWidget(self.save_coordinate_button)

        self.clear_button = QPushButton('清除定位点')
        self.clear_button.clicked.connect(self.clear_last_distance)
        self.button_layout.addWidget(self.clear_button)

        self.clear_yaw_button = QPushButton('清除yaw角度')
        self.clear_yaw_button.clicked.connect(self.clear_last_yaw)
        self.button_layout.addWidget(self.clear_yaw_button)

        self.clear_coordinate_button = QPushButton('清除坐标')
        self.clear_coordinate_button.clicked.connect(self.clear_last_coordinate)
        self.button_layout.addWidget(self.clear_coordinate_button)

        self.csv_button = QPushButton('csv转yaml')
        self.csv_button.clicked.connect(self.confirm_csv_to_yaml)
        self.button_layout.addWidget(self.csv_button)



        self.main_layout.addLayout(self.button_layout)

        # csv数据显示
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        self.csv_data_label = QLabel('CSV中的数据将显示在此处...')
        self.csv_data_label.setAlignment(Qt.AlignTop)  # 文字从顶部开始显示
        self.csv_data_label.setWordWrap(True)

        csv_label_style = """
            QLabel {
                padding: 12px;
                font-size: 12px;
                font-weight: bold;
                color: blue;
                border: 3px solid black;
            }
        """
        self.csv_data_label.setStyleSheet(csv_label_style)

        self.scroll_area.setWidget(self.csv_data_label)
        screen = QDesktopWidget().availableGeometry()  
        self.scroll_area.setFixedHeight(int(0.5 * screen.height())) 

        self.info_scroll_area = QScrollArea()
        self.info_scroll_area.setWidgetResizable(True)

        # 日志信息显示
        self.info_label = QLabel('打印信息将在此处显示...')
        self.info_label.setAlignment(Qt.AlignTop)
        self.info_label.setWordWrap(True)

        info_label_style = """
            QLabel {
                padding: 12px;
                font-size: 12px;
                font-weight: bold;
                color: green;
                border: 3px solid black;
            }
        """
        self.info_label.setStyleSheet(info_label_style)
        self.info_scroll_area.setWidget(self.info_label)
        screen = QDesktopWidget().availableGeometry()  
        self.info_scroll_area.setFixedHeight(int(0.5 * screen.height()))

        self.csv_layout = QHBoxLayout()
        # 三分之一
        self.csv_layout.addWidget(self.scroll_area, 5)
        # 三分之二
        self.csv_layout.addWidget(self.info_scroll_area, 3)

        self.main_layout.addLayout(self.csv_layout)
        self.main_layout.addStretch()
        self.setLayout(self.main_layout)
        self.update_distances_signal.connect(self.update_labels)
        self.update_code_signal.connect(self.update_code_labels)
        self.update_position_signal.connect(self.update_position_labels)
        self.update_yaw_signal.connect(self.update_yaw_label)

        self.front_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('robcup')  # 替换为你的包名

        # 订阅雷达信息
        rospy.Subscriber('/lidar_distances', Float32MultiArray, self.scan_callback) 
        rospy.Subscriber('/gm65_data', String, self.barcode_callback)
        # 订阅机器人坐标信息
        rospy.Subscriber('/tf_xy', Point, self.position_callback)
        # 订阅机器人yaw角度信息
        rospy.Subscriber('/lidar_yaw', Float32, self.yaw_callback) 

        
    def barcode_callback(self, msg):
        res = msg.data
        if res == '1' or res == '3':
            self.update_info_display(f"二维码信息：{res}")
            self.qrcode = res
            self.update_code_signal.emit(str(res),"","")
        else:
            self.update_info_display(f"条形码信息：{res}")
            if self.qrcode == '1':
                if self.first_code == '尚未可知':
                    self.first_code = res
                    self.update_code_signal.emit("",str(res),"")
                else:
                    if self.first_code != res:
                        self.third_code = res
                        self.update_code_signal.emit("","",str(res))
            else:
                if self.third_code == '尚未可知':
                    self.third_code = res
                    self.update_code_signal.emit("","",str(res))
                else:
                    if self.third_code != res:
                        self.first_code = res
                        self.update_code_signal.emit("",str(res),"")

    def scan_callback(self, msg):
        if len(msg.data) == 3:
            front_distance, left_distance, right_distance = msg.data
            self.front_distance = front_distance
            self.left_distance = left_distance
            self.right_distance = right_distance
            
            # 发射信号更新 UI
            self.update_distances_signal.emit(front_distance, left_distance, right_distance)

    def position_callback(self, msg):
        """处理机器人坐标信息的回调函数"""
        self.robot_x = msg.x
        self.robot_y = msg.y
        
        # 发射信号更新 UI
        self.update_position_signal.emit(self.robot_x, self.robot_y)

    def yaw_callback(self, msg):
        """处理机器人yaw角度信息的回调函数"""
        self.robot_yaw = msg.data
        
        # 发射信号更新 UI
        self.update_yaw_signal.emit(self.robot_yaw)


    def update_labels(self, front_distance, left_distance, right_distance):
        self.front_label.setText(
            f'前方: <span style="color:green;">{front_distance:.2f} m</span>')
        self.left_label.setText(
            f'左侧: <span style="color:green;">{left_distance:.2f} m</span>')
        self.right_label.setText(
            f'右侧: <span style="color:green;">{right_distance:.2f} m</span>')
        
    def update_code_labels(self, qrcode, first_code, third_code):
        if qrcode != "":
            self.qrcode_label.setText(
                f'二维码: <span style="color:orange;">{qrcode} </span>')
        if first_code != "":
            self.first_label.setText(
                f'1号病床: <span style="color:orange;">{first_code} </span>')
        if third_code != "":
            self.third_label.setText(
                f'3号病床: <span style="color:orange;">{third_code} </span>')

    def update_position_labels(self, x, y):
        """更新机器人坐标标签显示"""
        self.x_label.setText(
            f'X坐标: <span style="color:blue;">{x:.3f}</span>')
        self.y_label.setText(
            f'Y坐标: <span style="color:blue;">{y:.3f}</span>')

    def update_yaw_label(self, yaw):
        """更新机器人yaw角度标签显示"""
        self.yaw_value_label.setText(
            f'Yaw角度: <span style="color:purple;">{yaw:.2f}°</span>')

    def save_distances_to_file(self):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        if not os.path.exists(self.package_path+'/data'):
            os.makedirs(self.package_path+'/data')
        if self.front_distance <= 0.2 or self.left_distance <= 0.2 or self.right_distance <= 0.2 or self.front_distance == float('inf') or self.left_distance == float('inf') or self.right_distance == float('inf'):
            self.update_info_display(f"保存定位点操作无效，距离小于0.2m")
            QMessageBox.information(self, 'error', '保存定位点失败啦，请重新尝试哦', QMessageBox.Ok)
            return

        with open(self.package_path+'/data/distance_data.csv', 'a') as f:
            f.write(
                f'{current_time},{self.front_distance:.2f},{self.left_distance:.2f},{self.right_distance:.2f}\n')

        print(f"距离数据已保存至 data/distance_data.csv 文件中，时间为 {current_time}")
        self.update_csv_display()
        self.update_info_display(f"距离数据已保存，时间为 {current_time}")

    def save_yaw_to_file(self):
        """保存yaw角度到文件"""
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        if not os.path.exists(self.package_path+'/data'):
            os.makedirs(self.package_path+'/data')
        
        # 检查yaw角度数据是否有效
        if self.robot_yaw is None:
            self.update_info_display(f"保存yaw角度操作无效，yaw角度数据为空")
            QMessageBox.information(self, 'error', '保存yaw角度失败，yaw角度数据为空', QMessageBox.Ok)
            return

        with open(self.package_path+'/data/yaw_data.csv', 'a') as f:
            f.write(f'{current_time},{self.robot_yaw:.2f}\n')

        print(f"yaw角度数据已保存至 data/yaw_data.csv 文件中，时间为 {current_time}")
        self.update_yaw_csv_display()
        self.update_info_display(f"yaw角度数据已保存，时间为 {current_time}")

    def save_coordinate_to_file(self):
        """保存机器人x,y坐标到文件"""
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        if not os.path.exists(self.package_path+'/data'):
            os.makedirs(self.package_path+'/data')
        
        # 检查坐标数据是否有效
        if self.robot_x is None or self.robot_y is None:
            self.update_info_display(f"保存坐标操作无效，坐标数据为空")
            QMessageBox.information(self, 'error', '保存坐标失败，坐标数据为空', QMessageBox.Ok)
            return

        with open(self.package_path+'/data/coordinate_data.csv', 'a') as f:
            f.write(f'{current_time},{self.robot_x:.3f},{self.robot_y:.3f}\n')

        print(f"坐标数据已保存至 data/coordinate_data.csv 文件中，时间为 {current_time}")
        self.update_coordinate_csv_display()
        self.update_info_display(f"坐标数据已保存，时间为 {current_time}")

    def clear_last_distance(self):
        with open(self.package_path+'/data/distance_data.csv', 'r') as f:
            lines = f.readlines()

        if len(lines) >= 1:
            with open(self.package_path+'/data/distance_data.csv', 'w') as f:
                f.writelines(lines[:-1])
            print("已清除最后一组距离数据")
            self.update_csv_display()
            self.update_info_display("已清除最后一组距离数据")
        else:
            print("没有数据可以清除")
            self.update_info_display("没有数据可以清除")

    def clear_last_yaw(self):
        """清除最后一条yaw角度数据"""
        yaw_file = self.package_path+'/data/yaw_data.csv'
        try:
            with open(yaw_file, 'r') as f:
                lines = f.readlines()

            if len(lines) >= 1:
                with open(yaw_file, 'w') as f:
                    f.writelines(lines[:-1])
                print("已清除最后一组yaw角度数据")
                self.update_csv_display()
                self.update_info_display("已清除最后一组yaw角度数据")
            else:
                print("没有yaw角度数据可以清除")
                self.update_info_display("没有yaw角度数据可以清除")
        except FileNotFoundError:
            print("yaw角度数据文件不存在")
            self.update_info_display("yaw角度数据文件不存在")

    def clear_last_coordinate(self):
        """清除最后一条坐标数据"""
        coordinate_file = self.package_path+'/data/coordinate_data.csv'
        try:
            with open(coordinate_file, 'r') as f:
                lines = f.readlines()

            if len(lines) >= 1:
                with open(coordinate_file, 'w') as f:
                    f.writelines(lines[:-1])
                print("已清除最后一组坐标数据")
                self.update_csv_display()
                self.update_info_display("已清除最后一组坐标数据")
            else:
                print("没有坐标数据可以清除")
                self.update_info_display("没有坐标数据可以清除")
        except FileNotFoundError:
            print("坐标数据文件不存在")
            self.update_info_display("坐标数据文件不存在")

    # 创建确认对话框的函数
    def confirm_csv_to_yaml(self):
        # 创建一个消息框
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Question)
        msg_box.setWindowTitle("确认")
        msg_box.setText("真的要将CSV转为YAML吗?阁下三思啊!")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        
        # 获取用户的选择
        response = msg_box.exec_()
        
        # 如果用户选择了 "Yes"
        if response == QMessageBox.Yes:
            self.csv_to_yaml()  # 执行转换操作

    def csv_to_yaml_array(self, csv_file, yaml_file):
        """将CSV数据转换为三个独立的YAML文件"""
        success_files = []
        
        # 1. 处理雷达距离数据
        points = []
        if os.path.exists(csv_file):
            with open(csv_file, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if len(row) >= 4:
                        # 忽略第一列（日期时间），只取后面的数值
                        point = [float(row[1]), float(row[2]), float(row[3])]
                        points.append(point)
            
            if points:
                distance_yaml = self.package_path + "/data/distance.yaml"
                distance_dict = {'lader_point': points}
                with open(distance_yaml, 'w') as file:
                    yaml.dump(distance_dict, file, default_flow_style=False)
                success_files.append("distance.yaml")
        
        # 2. 处理yaw角度数据
        yaw_angles = []
        yaw_file = self.package_path + "/data/yaw_data.csv"
        if os.path.exists(yaw_file):
            with open(yaw_file, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if len(row) >= 2:
                        # 忽略第一列（日期时间），只取yaw角度值
                        yaw_angles.append(float(row[1]))
            
            if yaw_angles:
                yaw_yaml = self.package_path + "/data/yaw_angles.yaml"
                yaw_dict = {'yaw_angles': yaw_angles}
                with open(yaw_yaml, 'w') as file:
                    yaml.dump(yaw_dict, file, default_flow_style=False)
                success_files.append("yaw_angles.yaml")

        # 3. 处理坐标数据
        coordinates = []
        coordinate_file = self.package_path + "/data/coordinate_data.csv"
        if os.path.exists(coordinate_file):
            with open(coordinate_file, mode='r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if len(row) >= 3:
                        # 忽略第一列（日期时间），只取x,y坐标值
                        coordinate = [float(row[1]), float(row[2])]
                        coordinates.append(coordinate)
            
            if coordinates:
                coordinate_yaml = self.package_path + "/data/coordinates.yaml"
                coordinate_dict = {'coordinates': coordinates}
                with open(coordinate_yaml, 'w') as file:
                    yaml.dump(coordinate_dict, file, default_flow_style=False)
                success_files.append("coordinates.yaml")
        
        return success_files

    def csv_to_yaml(self):
        # 使用rospkg获取包路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robcup')  # 替换为你的包名
        # 获取CSV文件路径和YAML文件输出路径
        csv_file = package_path+"/data/distance_data.csv"
        yaml_file = package_path+"/data/distance.yaml"  # 这个参数现在不使用，但保持兼容性

        try:
            success_files = self.csv_to_yaml_array(csv_file, yaml_file)
            if success_files:
                files_str = "、".join(success_files)
                self.update_info_display(f"CSV转YAML成功！生成文件：{files_str}")
            else:
                self.update_info_display("CSV转YAML失败：没有有效数据可转换")
        except Exception as e:
            self.update_info_display(f"CSV转YAML失败：{str(e)}")

    def update_csv_display(self):
        """更新CSV数据显示，同时显示距离和yaw数据"""
        try:
            # 读取距离数据
            distance_text = ""
            try:
                with open(self.package_path+'/data/distance_data.csv', 'r') as f:
                    csv_lines = f.readlines()
                    distance_lines = []
                    for line in csv_lines:
                        parts = line.strip().split(',')
                        if len(parts) == 4:
                            time, front, left, right = parts
                            formatted_line = f'{time}  前方: {front} m  左侧: {left} m  右侧: {right} m'
                            distance_lines.append(formatted_line)
                    if distance_lines:
                        distance_text = "=== 距离数据 ===\n" + '\n'.join(distance_lines)
            except:
                pass

            # 读取yaw数据
            yaw_text = ""
            try:
                with open(self.package_path+'/data/yaw_data.csv', 'r') as f:
                    csv_lines = f.readlines()
                    yaw_lines = []
                    for line in csv_lines:
                        parts = line.strip().split(',')
                        if len(parts) == 2:
                            time, yaw = parts
                            formatted_line = f'{time}  Yaw角度: {yaw}°'
                            yaw_lines.append(formatted_line)
                    if yaw_lines:
                        yaw_text = "=== Yaw角度数据 ===\n" + '\n'.join(yaw_lines)
            except:
                pass

            # 读取坐标数据
            coordinate_text = ""
            try:
                with open(self.package_path+'/data/coordinate_data.csv', 'r') as f:
                    csv_lines = f.readlines()
                    coordinate_lines = []
                    for line in csv_lines:
                        parts = line.strip().split(',')
                        if len(parts) == 3:
                            time, x, y = parts
                            formatted_line = f'{time}  X: {x}  Y: {y}'
                            coordinate_lines.append(formatted_line)
                    if coordinate_lines:
                        coordinate_text = "=== 坐标数据 ===\n" + '\n'.join(coordinate_lines)
            except:
                pass

            # 合并显示
            combined_text = ""
            if distance_text:
                combined_text += distance_text
            if yaw_text:
                if combined_text:
                    combined_text += "\n\n"
                combined_text += yaw_text
            if coordinate_text:
                if combined_text:
                    combined_text += "\n\n"
                combined_text += coordinate_text
            
            if combined_text:
                self.csv_data_label.setText(combined_text)
            else:
                self.csv_data_label.setText("CSV文件为空")

            # 强制处理挂起的事件，确保 QLabel 的内容完全更新
            QApplication.processEvents()

            # 自动滚动到底部
            self.scroll_area.verticalScrollBar().setValue(
                self.scroll_area.verticalScrollBar().maximum())
        except Exception as e:
            self.csv_data_label.setText(f"读取CSV文件失败: {str(e)}")

    def update_yaw_csv_display(self):
        """更新yaw角度CSV数据显示，调用通用的update_csv_display"""
        self.update_csv_display()

    def update_coordinate_csv_display(self):
        """更新坐标CSV数据显示，调用通用的update_csv_display"""
        self.update_csv_display()

    def update_info_display(self, info_message):
        """更新打印信息的显示内容"""
        current_text = self.info_label.text()
        new_text = f"{current_text}\n{info_message}"
        self.info_label.setText(new_text)

        # 强制处理挂起的事件，确保 QLabel 的内容完全更新
        QApplication.processEvents()

        # 自动滚动到底部
        self.info_scroll_area.verticalScrollBar().setValue(
            self.info_scroll_area.verticalScrollBar().maximum())

def main():
    rospy.init_node('laser_display_node', anonymous=True)

    app = QApplication(sys.argv)
    display = LaserDisplay()
    display.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()