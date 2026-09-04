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
import os
from datetime import datetime
import numpy as np
import rospkg
class LaserDisplay(QWidget):

    update_distances_signal = pyqtSignal(float, float, float)
    update_code_signal = pyqtSignal(str, str, str)

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

        self.qrcode = "尚未可知"
        self.first_code = "尚未可知"
        self.third_code = "尚未可知"

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

        self.clear_button = QPushButton('清除定位点')
        self.clear_button.clicked.connect(self.clear_last_distance)
        self.button_layout.addWidget(self.clear_button)

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

        self.front_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('robcup')  # 替换为你的包名

        # 订阅雷达信息
        rospy.Subscriber('/lidar_distances', Float32MultiArray, self.scan_callback) 
        rospy.Subscriber('/gm65_data', String, self.barcode_callback) 

        
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
        points = []
        with open(csv_file, mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                # 忽略第一列（日期时间），只取后面的数值
                point = [float(row[1]), float(row[2]), float(row[3])]
                points.append(point)
        
        # 生成字典数据，符合YAML的结构
        data_dict = {'lader_point': points}
        
        # 将字典数据写入到YAML文件
        with open(yaml_file, 'w') as file:
            yaml.dump(data_dict, file, default_flow_style=False)

    def csv_to_yaml(self):
        # 使用rospkg获取包路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robcup')  # 替换为你的包名
        # 获取CSV文件路径和YAML文件输出路径
        csv_file = package_path+"/data/distance_data.csv"
        yaml_file = package_path+"/data/distance.yaml"

        try:
            self.csv_to_yaml_array(csv_file, yaml_file)
            self.update_info_display("csv转yaml格式成功")
        except Exception as e:
            self.update_info_display("csv转yaml格式失败")

    def update_csv_display(self):
        try:
            with open(self.package_path+'/data/distance_data.csv', 'r') as f:
                csv_lines = f.readlines()

                formatted_lines = []
                for line in csv_lines:
                    parts = line.strip().split(',')
                    if len(parts) == 4:
                        time, front, left, right = parts
                        formatted_line = f'{time}  前方: {front} m  左侧: {left} m  右侧: {right} m'
                        formatted_lines.append(formatted_line)

                if formatted_lines:
                    self.csv_data_label.setText('\n'.join(formatted_lines))
                else:
                    self.csv_data_label.setText("CSV文件为空")

                # 强制处理挂起的事件，确保 QLabel 的内容完全更新
                QApplication.processEvents()

                # 自动滚动到底部
                self.scroll_area.verticalScrollBar().setValue(
                    self.scroll_area.verticalScrollBar().maximum())
        except Exception as e:
            self.csv_data_label.setText(f"读取CSV文件失败: {str(e)}")

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