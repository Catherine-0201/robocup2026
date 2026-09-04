#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String

def serial_node():
    # 初始化ROS节点
    rospy.init_node('gm65_node', anonymous=True)
    
    # 获取参数，串口设备名称和波特率
    port = '/dev/gm65'  # 串口设备
    baud_rate = 9600  # 波特率
    topic_name = "/gm65_data"  # 发布的ROS话题名称
    
    # 创建发布者，发布到指定话题
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    
    try:
        # 打开串口
        ser = serial.Serial(port, baud_rate, timeout=1)
        rospy.loginfo(f"Serial port {port} opened at baud rate {baud_rate}")
        buffer=""
        # 持续接受串口数据并发布
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                # 读取串口数据
                data = ser.read().decode('utf-8')
                buffer+=data
                if '\r' in buffer:
                    line = buffer.strip('\r')
                    buffer = ""
                    pub.publish(line)
                    print("Receive:",line)
                    if(line=='1' or line=='3'):
                        rospy.set_param("gm65_data", int(line))
                # 将数据发布到ROS话题
                
    except serial.SerialException as e:
        rospy.logerr(f"Error opening serial port: {e}")
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed.")

if __name__ == '__main__':
    try:
        serial_node()
    except rospy.ROSInterruptException:
        pass
