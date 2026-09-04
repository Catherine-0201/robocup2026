#!/usr/bin/env python

import csv
import yaml
import rospy
import rospkg

def csv_to_yaml_array(csv_file, yaml_file):
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

    rospy.loginfo(f"CSV data has been converted to YAML format and saved to {yaml_file}")

if __name__ == '__main__':
    rospy.init_node('csv_to_yaml_node', anonymous=True)
    # 使用rospkg获取包路径
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robcup')  # 替换为你的包名
    # 获取CSV文件路径和YAML文件输出路径
    csv_file = package_path+"/data/distance_data.csv"
    yaml_file = package_path+"/data/distance.yaml"

    try:
        csv_to_yaml_array(csv_file, yaml_file)
    except Exception as e:
        rospy.logerr(f"Error converting CSV to YAML: {e}")
