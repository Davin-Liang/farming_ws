#!/usr/bin/env python3
import math
from math import copysign
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets # type: ignore
import os
import yaml
from std_msgs.msg import String, Bool, Int16MultiArray

class Farming_visioner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        self.area_scaling_factor = 0.5
        self.file_path = os.path.expanduser('~/farming_ws/src/motion_controller/config/position_point.yaml') # TODO:
        self.load_config_file_() # 加载 arm 参数
        self.reset_vision_detect = False # 是否重置视觉数据
        self.open_vision_detect = False # 是否打开视觉检测
        self.pre_process = False # 是否打开数据预处理
        self.flowers_with_tag = []
        self.flowers_with_tag_again = []
        self.O_distance_threthold_of_judge_same_goal = 200
        self.central_point_of_camera = [520, 480]
        self.area_of_polliating = 400 # 
        self.joint_speed = 4 # 将关节的转动的角度当作速度
        self.thretholds_of_joint_moving = {'x_error': 5, 'y_error': 5, 'area_error': 50}
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Int16MultiArray, "joint_angles", 10)
        self.angles_of_joints = Int16MultiArray()

    def vision_callback_(self, msg):
        """ 视觉回调函数 """
        if not self.open_vision_detect:
            return

        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0} # 类型、中心点坐标、面积
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                if msg.targets[i].type == "male": 
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.y_offset + msg.targets[0].rois[0].rect.width/2)
                    flower['Area'] = flower['CentralPoint'][0] * flower['CentralPoint'][1] * self.area_scaling_factor
                elif msg.targets[i].type == "female":
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.y_offset + msg.targets[0].rois[0].rect.width/2)
                    flower['Area'] = flower['CentralPoint'][0] * flower['CentralPoint'][1] * self.area_scaling_factor

                # 得到原始数据
                flowers_lists.append(flower)

        if 0 != len(flowers_lists): # 预防处理空数据
            self.confrim_moving_goal_for_arm(flowers_lists)

    def confrim_moving_goal_for_arm(self, flowers_lists):
        """ 确定 arm 的移动目标 """
        # 数据预处理并选择第一个处理的目标
        self.data_pre_processing(flowers_lists)

        # 更新数据
        for flower in flowers_lists:
            for index, flower_with_tag in enumerate(self.flowers_with_tag):
                if self.calculate_O_distance(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                    self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                    break # 跳出内层 for 循环
        
        # 控制 arm
        self.control_arm()

    def control_arm(self):
        y_error = 0
        x_error = 0
        area_error = 0
        for flower_with_tag in self.flowers_with_tag:
            if flower_with_tag['Moving'] == True:
                x_error = flower_with_tag['CentralPoint'][0] - self.central_point_of_camera[0]
                y_error = flower_with_tag['CentralPoint'][1] - self.central_point_of_camera[1]
                area_error = flower_with_tag['Area'] - self.area_of_polliating
                break
        self.angles_of_joints.data.clear()
        if abs(x_error) > self.thretholds_of_joint_moving['x_error']:
            self.arm_params['joint1'] = self.limit_num(self.arm_params['joint1'] + copysign(self.joint_speed, x_error), self.arm_params['joint1_limiting'])
            self.angles_of_joints.data.append(self.arm_params['joint1'])
        if abs(area_error) > self.thretholds_of_joint_moving['area_error']:
            self.arm_params['joint2'] = self.limit_num(self.arm_params['joint2'] + copysign(self.joint_speed, area_error), self.arm_params['joint2_limiting'])
            self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        if abs(y_error) > self.thretholds_of_joint_moving['y_error']:
            self.arm_params['joint4'] = self.limit_num(self.arm_params['joint4'] + copysign(self.joint_speed, y_error), self.arm_params['joint4_limiting'])
            self.angles_of_joints.data.append(self.arm_params['joint4'])
        if (abs(x_error) < self.thretholds_of_joint_moving['x_error'] and
            abs(area_error) < self.thretholds_of_joint_moving['area_error'] and
            abs(y_error) < self.thretholds_of_joint_moving['y_error']):
            for index, flowers_with_tag in enumerate(self.flowers_with_tag_again):
                if flowers_with_tag['Moving'] == True:
                    self.flowers_with_tag_again[index]['Moving'] = False
                    self.flowers_with_tag_again[index]['Pollinated'] = True
            self.reset_arm_default_pose()
        self.joint_angles_publisher_.publish(self.angles_of_joints)

    def reset_arm_default_pose(self):
        """ 控制 arm 回到初始姿态 """
        self.arm_params['joint1'] = self.arm_params['joint1_default']
        self.arm_params['joint2'] = self.arm_params['joint2_default']
        self.arm_params['joint3'] = self.arm_params['joint3_default']
        self.arm_params['joint4'] = self.arm_params['joint4_default']
        self.open_vision_detect = False
        self.pre_process = True
        
    
    def limit_num(self, num, num_range):
        if num > num_range[1]:
            num = num_range[1]
        elif num < num_range[0]:
            num = num_range[0]
        return num

    def data_pre_processing(self, flowers_lists):
        """ 为存储花属性的字典添加花属性：是否正在操作、是否已授粉 """
        # 类型、中心的坐标、面积、是否正在操作、是否已授粉
        flower_with_tag = {'Type': '', 'CentralPoint': [], 'Area': 0, 'Moving': False, 'Pollinated': False}
        if self.pre_process:
            if len(self.flowers_with_tag) == 0:
                for index, flower in enumerate(flowers_lists):
                    flower_with_tag['Type'] = flower['Type']
                    flower_with_tag['CentralPoint'] = flower['CentralPoint']
                    flower_with_tag['Area'] = flower['Area']
                    if index == 0:
                        flower_with_tag['Moving'] = True
                    self.flowers_with_tag.append(flower_with_tag)
                self.flowers_with_tag_again = self.flowers_with_tag
            else:
                self.flowers_with_tag = self.flowers_with_tag_again
                # 更新中心点坐标参数
                for flower in flowers_lists:
                    for index, flower_with_tag in enumerate(self.flowers_with_tag):
                        if self.calculate_O_distance(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                            self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                            break
                # 寻找未“授粉”的花，找到的第一朵就设置为目标
                for index, flower_with_tag in enumerate(self.flowers_with_tag):
                    if flower_with_tag['Pollinated'] != True:
                        self.flowers_with_tag[index]['Moving'] = True
                        break
        self.pre_process = False # 关闭数据预处理  

    def calculate_O_distance(self, point1, point2):
        """ 计算两个坐标点之间的 O 式距离 """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def load_config_file_(self):
        """ 读取 YAML 文件 """
        with open(self.file_path, 'r') as file:
            self.arm_params = yaml.safe_load(file) 



def main():
    rclpy.init()
    try:
        node = Farming_visioner("Farming_visioner")


        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()