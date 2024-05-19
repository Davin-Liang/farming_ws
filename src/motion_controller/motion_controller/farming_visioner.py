#!/usr/bin/env python3
import math
from math import copysign
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets # type: ignore
import os
import yaml
from std_msgs.msg import String, Bool, Int16MultiArray
from  threading import Thread
import time
import subprocess
import copy

class Farming_visioner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        # 加载 arm 参数
        self.file_path = os.path.expanduser('~/farming_ws/src/farming_vision/config/arm_params.yaml') # TODO:
        self.load_config_file_()
        # 重要 BOOL 值
        self.open_vision_detect = False # 是否打开视觉检测
        self.pre_process = False # 是否打开数据预处理
        self.debug_mode = False
        self.reset_arm = False
        self.active_thread = False
        # 数据字典
        self.flowers_with_tag = [] # 存储花属性
        self.flowers_with_tag_again = [] # 存储花属性
        self.arm_params = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0} # 存储实时的机械臂角度
        # 可调参数
        self.area_scaling_factor = 0.5 # 面积缩放系数
        self.O_distance_threthold_of_judge_same_goal = 200 # 判断前后两次数据检测的识别框是否为同一个目标的阈值
        self.central_point_of_camera = [520, 480] # 相机中心点
        self.area_of_polliating = 400 # 识别框为多少时才进行授粉的面积阈值
        self.joint_speed = 3 # 关节转动速度，将关节的转动的角度当作速度
        self.threthold_of_x_error = 5.0
        self.threthold_of_y_error = 5.0
        self.threthold_of_area_error = 50.0
        self.thretholds_of_joint_moving = {'x_error': 5, 'y_error': 5, 'area_error': 50}
        # 使用到的订阅者和发布者
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Int16MultiArray, "joint_angles", 10)
        self.angles_of_joints = Int16MultiArray()

        # self.set_new_param_to_ros('reset_vision_detect', 'bool')
        self.set_new_param_to_ros('debug_mode', 'bool')
        self.set_new_param_to_ros('reset_arm', 'bool')
        self.set_new_param_to_ros('area_scaling_factor')
        self.set_new_param_to_ros('O_distance_threthold_of_judge_same_goal')
        self.set_new_param_to_ros('area_of_polliating')
        self.set_new_param_to_ros('joint_speed')
        self.set_new_param_to_ros('threthold_of_x_error')
        self.set_new_param_to_ros('threthold_of_y_error')
        self.set_new_param_to_ros('threthold_of_area_error')

        self.param_timer = self.create_timer(0.04, self.param_timer_work_)
    
    # ---------------- 对外接口函数 -----------------
    def vision_control_arm(self, pose_name):
        self.pose_name = pose_name
        self.arm_params['joint1'] = self.default_arm_params['joint1_'+self.pose_name]
        self.arm_params['joint2'] = self.default_arm_params['joint2_'+self.pose_name]
        self.arm_params['joint3'] = self.default_arm_params['joint3_'+self.pose_name]
        self.arm_params['joint4'] = self.default_arm_params['joint4_'+self.pose_name]
        self.angles_of_joints.data.clear()
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        self.open_vision_detect     = True
        self.pre_process            = True
        self.reset_vision_data()
        # 堵塞函数直到完成任务
        while self.open_vision_detect:
            pass
        self.active_thread = False
        if self.debug_mode: # 完成目标后，自动将 self.debug_mode 参数置 false
            all_new_parameters = []
            self.debug_mode = rclpy.parameter.Parameter('debug_mode', rclpy.Parameter.Type.DOUBLE, False)
            all_new_parameters.append(self.debug_mode)
            self.set_parameters(all_new_parameters)

    def reset_vision_data(self):
        self.flowers_with_tag.clear()
        self.flowers_with_tag_again.clear()

    def find_next_arm_goal_on_position(self):
        self.open_vision_detect = True
        self.reset_vision_detect = False
        # 堵塞函数直到完成任务
        while self.open_vision_detect:
            pass
    # ----------------------------------------------


    def param_timer_work_(self):
        self.update_params_()
        if self.reset_arm:
            self.reset_arm_default_pose()
        if self.debug_mode and self.active_thread == False:
            print("正在单点测试")
            self.debug_thread = Thread(target=self.vision_control_arm, args=('a_left',))
            self.active_thread = True
            # self.vision_control_arm('a_left')


    def add_variable(self, var_name, value):
        """ 通过一个字符串创建一个类中变量 """
        setattr(self, var_name, value)
    
    def get_variable(self, var_name):
        """ 通过一个字符串读取类中变量的值 """
        return getattr(self, var_name, None)
    
    def update_variable(self, var_name, new_value):
        """ 通过一个字符串更新一个类中变量的值 """
        setattr(self, var_name, new_value)

    def set_new_param_to_ros(self, var_name, data_type='double'):
        """ 通过一个字符串向参数服务器定义一个值，并读取该值 """
        if data_type == 'bool':
            self.declare_parameter(var_name, self.get_variable(var_name))
            self.update_variable(var_name, self.get_parameter(var_name).get_parameter_value().bool_value)
        if data_type == 'double':
            self.declare_parameter(var_name, self.get_variable(var_name))
            self.update_variable(var_name, self.get_parameter(var_name).get_parameter_value().double_value)
        if data_type == 'string':
            self.declare_parameter(var_name, self.get_variable(var_name))
            self.update_variable(var_name, self.get_parameter(var_name).get_parameter_value().string_value)

    def update_params_from_ros(self, var_name, data_type='double'):
        if data_type == 'bool':
            self.update_variable(var_name, self.get_parameter(var_name).get_parameter_value().bool_value)
        if data_type == 'double':
            self.update_variable(var_name, self.get_parameter(var_name).get_parameter_value().double_value)
        if data_type == 'string':
            self.update_variable(var_name, self.get_parameter(var_name).get_parameter_value().string_value)

    def update_params_(self):
        self.update_params_from_ros('area_scaling_factor')
        self.update_params_from_ros('O_distance_threthold_of_judge_same_goal')
        self.update_params_from_ros('area_of_polliating')
        self.update_params_from_ros('joint_speed')
        self.update_params_from_ros('threthold_of_x_error')
        self.update_params_from_ros('threthold_of_y_error')
        self.update_params_from_ros('threthold_of_area_error')
        self.update_params_from_ros('debug_mode')
        self.update_params_from_ros('reset_arm')

    def vision_callback_(self, msg):
        """ 视觉回调函数 """
        if not self.open_vision_detect:
            return
        print("正在通过视觉控制机械臂")

        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0} # 类型、中心点坐标、面积
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                if msg.targets[i].type == "male": 
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    flower['Area'] = flower['CentralPoint'][0] * flower['CentralPoint'][1] * self.area_scaling_factor
                elif msg.targets[i].type == "female":
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    flower['Area'] = flower['CentralPoint'][0] * flower['CentralPoint'][1] * self.area_scaling_factor

                # 得到原始数据
                flowers_lists.append(copy.deepcopy(flower)) # 深拷贝

        if 0 != len(flowers_lists): # 预防处理空数据
            self.confrim_moving_goal_for_arm(flowers_lists)

        time.sleep(0.1)

    def confrim_moving_goal_for_arm(self, flowers_lists):
        """ 确定 arm 的移动目标 """
        # 数据预处理并选择第一个处理的目标
        self.data_pre_processing(flowers_lists)

        # 更新数据
        print("正在更新数据")
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
                x_error = self.central_point_of_camera[0] - flower_with_tag['CentralPoint'][0]
                y_error = self.central_point_of_camera[1] - flower_with_tag['CentralPoint'][1]
                area_error = self.area_of_polliating - flower_with_tag['Area']
                break
        self.angles_of_joints.data.clear()
        if abs(x_error) > self.threthold_of_x_error:
            self.arm_params['joint1'] = self.limit_num(self.arm_params['joint1'] + copysign(self.joint_speed, -x_error), self.default_arm_params['joint1_limiting'])
            self.angles_of_joints.data.append(self.arm_params['joint1'])
        if abs(area_error) > self.threthold_of_area_error:
            self.arm_params['joint2'] = self.limit_num(self.arm_params['joint2'] + copysign(self.joint_speed, -area_error), self.default_arm_params['joint2_limiting'])
            self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        if abs(y_error) > self.threthold_of_y_error:
            self.arm_params['joint4'] = self.limit_num(self.arm_params['joint4'] + copysign(self.joint_speed, y_error), self.default_arm_params['joint4_limiting'])
            self.angles_of_joints.data.append(self.arm_params['joint4'])
        if (abs(x_error) < self.threthold_of_x_error and
            abs(area_error) < self.threthold_of_area_error and
            abs(y_error) < self.threthold_of_y_error):
            print("已经完成授粉")
            for index, flower_with_tag in enumerate(self.flowers_with_tag):
                if flower_with_tag['Moving'] == True:
                    self.flowers_with_tag_again[index]['Moving'] = False
                    self.flowers_with_tag_again[index]['Pollinated'] = True
            self.reset_arm_default_pose()
            return
        self.joint_angles_publisher_.publish(self.angles_of_joints)

    def reset_arm_default_pose(self):
        """ 控制 arm 回到初始姿态 """
        print("控制 arm 回到初始姿态")
        self.open_vision_detect = False
        self.pre_process = False
        all_new_parameters = []
        self.debug_mode = rclpy.parameter.Parameter('debug_mode', rclpy.Parameter.Type.DOUBLE, False)
        all_new_parameters.append(self.debug_mode)
        self.reset_arm = rclpy.parameter.Parameter('reset_arm', rclpy.Parameter.Type.DOUBLE, False)
        all_new_parameters.append(self.reset_arm)
        self.set_parameters(all_new_parameters)
        self.arm_params['joint1'] = self.default_arm_params['joint1_'+self.pose_name]
        self.arm_params['joint2'] = self.default_arm_params['joint2_'+self.pose_name]
        self.arm_params['joint3'] = self.default_arm_params['joint3_'+self.pose_name]
        self.arm_params['joint4'] = self.default_arm_params['joint4_'+self.pose_name]
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.joint_angles_publisher_.publish(self.angles_of_joints)
    
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
                print("正在进行数据预处理")
                male_num = 0
                female_num = 0
                for index, flower in enumerate(flowers_lists):
                    flower_with_tag['Type'] = flower['Type']
                    flower_with_tag['CentralPoint'] = flower['CentralPoint']
                    flower_with_tag['Area'] = flower['Area']
                    if index == 0:
                        flower_with_tag['Moving'] = True
                    self.flowers_with_tag.append(copy.deepcopy(flower_with_tag))

                    if flower['Type'] == 'male':
                        male_num += 1
                    elif flower['Type'] == 'female':
                        female_num += 1
                self.flowers_with_tag_again = self.flowers_with_tag

                # 语音播报
                self.voice_broadcast(male_num, female_num)
            else:
                print("正在授粉第二个目标点")
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

    def voice_broadcast(self, male_num, female_num):
        """ 语音播报 """
        if male_num == 0 and female_num == 3:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_0_3.wav', '-D', '0', '-d', '1'])
        elif male_num == 1 and female_num == 2:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_1_2.wav', '-D', '0', '-d', '1'])
        elif male_num == 2 and female_num == 1:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_2_1.wav', '-D', '0', '-d', '1'])
        elif male_num == 3 and female_num == 0:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_3_0.wav', '-D', '0', '-d', '1'])

    def calculate_O_distance(self, point1, point2):
        """ 计算两个坐标点之间的 O 式距离 """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def load_config_file_(self):
        """ 读取 YAML 文件 """
        with open(self.file_path, 'r') as file:
            self.default_arm_params = yaml.safe_load(file)



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
        if node.debug_thread is not None:
            node.debug_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()