#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets # type: ignore
import os
import yaml
from std_msgs.msg import Int32MultiArray
import time
import subprocess
import copy
from threading import Thread
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow, radians
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import PyKDL
from geometry_msgs.msg import Vector3
from pid import PID

class Game_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        # 加载 arm 参数
        self.file_path = os.path.expanduser('~/farming_ws/src/farming_vision/config/arm_params.yaml')
        self.load_config_file_()
    
        # 重要 BOOL 值
        self.open_vision_detect = False # 是否打开视觉检测
        self.pre_process = False # 是否打开数据预处理

        # 数据字典
        self.arm_params = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0} # 存储实时的机械臂角度
        self.flowers_with_tag = [] # 存储花属性
        self.flowers_with_tag_again = [] # 存储花属性
        # 可调参数
        self.area_scaling_factor = 0.25 # 面积缩放系数
        self.O_distance_threthold_of_judge_same_goal = 300 # 判断前后两次数据检测的识别框是否为同一个目标的阈值
        self.central_point_of_camera = [320, 240] # 相机中心点
        self.area_of_polliating = 70000 # 识别框为多少时才进行授粉的面积阈值
        self.joint_speed = 1.0 # 关节转动速度，将关节的转动的角度当作速度
        self.threthold_of_x_error = 10.0
        self.threthold_of_y_error = 5.0
        self.threthold_of_area_error = 5000.0
        self.servo_time = 200  #机械臂运动时间，单位mm
        self.servo_reset_time = 1000  #机械臂初始位置运动时间

        # 使用到的订阅者和发布者
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Int32MultiArray, "servo_commands", 10)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "laser", self.lidar_callback_, 10)
        self.angles_of_joints = Int32MultiArray()
        self.euler_angles_subscriber_ = self.create_subscription(Vector3, "euler_angles", self.euler_angles_callback_, 10)
        self.yaw_angle = 0.0

        self.move_cmd = Twist()

        self.ori_angle_pid = PID(0.685, 0.0, 0.426, 1.4, 0.0)
        self.distance_pid = PID(0.42, 0.0, 0.08, 1.0, 0.0)

        
        self.distance = 0.0
        self.angle = 0.0
        self.angle = radians(self.angle)
        self.deviation_angle = radians(0.85)
        self.liear_speed = 0.5
        self.distance_tolerance = 0.03
        self.angle_tolerance = radians(2.0)
        self.odom_linear_scale_correction = 1.0
        self.odom_angular_scale_correction = 1.0
        self.start_for_lidar_distance = False
        self.start_for_pid_distance = False
        self.base_frame = 'base_footprint'
        self.odom_frame = 'odom'

        self.lidar_threthold = 0.1

        self.flowers_lists = []
        
        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y

        self.distance_error = 0
        self.angle_error    = 0
        self.run_times = 0

        time.sleep(3.0)

        # 创建定时器
        self.work_timer = self.create_timer(0.04, self.timer_work_)
        self.arm_timer = self.create_timer(0.3, self.arm_timer_callback_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()


    # ---------------- 对外接口函数 -----------------
    def vision_control_arm(self, place_name, pose_name):
        self.place_name = place_name
        self.choose_arm_goal(pose_name)
        self.open_vision_detect     = True
        self.pre_process            = True
        self.arm_moving = False
        self.reset_vision_data()
        # 堵塞函数直到完成任务
        print("正在等待完成任务")
        while self.open_vision_detect:
            pass

    def reset_vision_data(self):
        self.flowers_with_tag.clear()
        self.flowers_with_tag_again.clear()

    def find_next_arm_goal_on_position(self):
        self.open_vision_detect = True
        self.pre_process = True
        # 堵塞函数直到完成任务
        while self.open_vision_detect:
            pass
    # ---------------

    # 废弃
    def control_car_turn(self, turn_speed, turn_time):
        """ 废弃 """
        self.move_cmd.angular.z = turn_speed
        time.sleep(turn_time)
        self.move_cmd.angular.z = 0.0

    def set_distance(self, distance):
        """ 设置车轮方向的行驶距离及以什么样的速度行驶 """
        self.distance = distance
        self.start_for_pid_distance = True
        # 等待完成任务
        while self.start_for_pid_distance:
            pass
        time.sleep(2.0)

    def set_angle(self, angle):
        """ 设置底盘转动角度 """
        self.angle = radians(angle)
        # 等待转完角度
        while abs(self.angle-self.get_odom_angle_())>self.angle_tolerance:
            pass
        time.sleep(2.0)

    def start_car_and_lidar_controls_stopping(self, speed, threthold=0.1, ignore_num=0):
        """ 开动车并使用单线激光控制小车停止 """
        self.liear_speed = speed
        self.lidar_threthold = threthold
        self.start_for_lidar_distance = True

        # 等待 car 到位
        time.sleep(3.5) # 保证 car 驶出激光遮挡区域
        for i in range(ignore_num+1):
            print("正在检测中")
            print("激光", self.lidar_distance)
            print("阈值", self.lidar_threthold)
            while self.lidar_distance > self.lidar_threthold:
                pass
        print("激光已经到达下一个激光遮挡区域")
        self.start_for_lidar_distance = False
        time.sleep(2.0)

    # 废弃
    def vision_choose_goal_in_A(self, pose_name):
        """ 传入视觉目标 """
        self.place_name = "A"
        self.pose_name = pose_name
        self.choose_arm_goal(pose_name)
        self.open_vision_detect     = True
        # 堵塞函数直到完成任务
        # print("正在等待完成任务")
        while self.open_vision_detect:
            pass

    # 废弃
    def vision_choose_goal_in_B(self, pose_name):
        self.place_name = "B"
        self.pose_name = pose_name
        self.run_times = 0
        if pose_name == "front":
            # 中间花
            self.choose_arm_goal("b_middle_front_pre")
            while self.open_vision_detect:
                pass
            
            self.choose_arm_goal("b_middle_front_pre")
            self.choose_arm_goal_in_number(joint1=126)
            self.choose_arm_goal("b_right_front_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal("b_right_front_pre")
            self.choose_arm_goal("b_left_front_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal("b_left_front_pre")
            self.choose_arm_goal_in_number(joint2=107, joint3=124, joint4=93)
            self.choose_arm_goal("b_middle_front_pre")
        if pose_name == "back":
            self.choose_arm_goal("b_middle_back_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal("b_middle_back_pre")
            self.choose_arm_goal_in_number(joint1=97)
            self.choose_arm_goal("b_left_back_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal("b_left_back_pre")
            self.choose_arm_goal("b_right_back_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal("b_right_back_pre")
            self.choose_arm_goal_in_number(joint2=107, joint3=124, joint4=93)
            self.choose_arm_goal("b_middle_back_pre")

    def arm_timer_callback_(self):
        print("正在等待开启视觉")
        if not self.open_vision_detect:
            return
        print("正在通过视觉控制机械臂")

        if 0 != len(self.flowers_lists): # 预防处理空数据
            print(self.flowers_lists)
            self.confrim_moving_goal_for_arm(self.flowers_lists)

    def euler_angles_callback_(self, msg):
        self.yaw_angle = msg.z        

    def vision_callback_(self, msg):
        """ 视觉回调函数 """
        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0} # 类型、中心点坐标、面积
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                flower['CentralPoint'].clear()
                if msg.targets[i].type == "male": 
                    flower['Type'] = msg.targets[i].type

                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    flower['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width
                elif msg.targets[i].type == "famale":
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    flower['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width

                # 得到原始数据

                flowers_lists.append(copy.deepcopy(flower)) # 深拷贝
        #print(flowers_lists)
        self.flowers_lists.clear()
        self.flowers_lists = copy.deepcopy(flowers_lists)

    def confrim_moving_goal_for_arm(self, flowers_lists):
        """ 确定 arm 的移动目标 """
        # 数据预处理并选择第一个处理的目标
        self.data_pre_processing(flowers_lists)
        # 更新数据
        # print("正在更新数据")
        for flower in flowers_lists:
            for index, flower_with_tag in enumerate(self.flowers_with_tag):
                if self.calculate_O_distance(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                    self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                    self.flowers_with_tag[index]['Area'] = flower['Area']
                    #print(self.flowers_with_tag)
                    break # 跳出内层 for 循环

        # # 控制 arm
        #self.control_arm()

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
                    if flower['Type'] == 'male':
                        flower_with_tag['Type'] = flower['Type']
                        flower_with_tag['CentralPoint'] = flower['CentralPoint']
                        flower_with_tag['Area'] = flower['Area']
                        if self.arm_moving == False:
                            flower_with_tag['Moving'] = True
                            self.arm_moving = True
                        else:
                            flower_with_tag['Moving'] = False
                        self.flowers_with_tag.append(copy.deepcopy(flower_with_tag))

                    # if flower['Type'] == 'male':
                    #     male_num += 1
                    # elif flower['Type'] == 'famale':
                    #     female_num += 1
                self.flowers_with_tag_again = self.flowers_with_tag
                #添加语音播报
                self.voice(flowers_lists)

            else:
                print("正在授粉第二个目标点")
                self.flowers_with_tag = self.flowers_with_tag_again
                # 更新中心点坐标参数
                for flower in flowers_lists:
                    for index, flower_with_tag in enumerate(self.flowers_with_tag):
                        if self.calculate_O_distance(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                            self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                            self.flowers_with_tag[index]['Area'] = flower['Area']
                            break
                #self.flowers_with_tag_again = self.flowers_with_tag    #更新again数据
                # 寻找未“授粉”的花，找到的第一朵就设置为目标
                for index, flower_with_tag in enumerate(self.flowers_with_tag):
                    if flower_with_tag['Pollinated'] != True:
                        self.flowers_with_tag[index]['Moving'] = True
                        break
        self.pre_process = False # 关闭数据预处理

    def control_arm(self):
        y_error = 0
        x_error = 0
        area_error = 0
        print(self.flowers_with_tag)
        for flower_with_tag in self.flowers_with_tag:
            if flower_with_tag['Moving'] == True:
                print("正在获取 x 轴误差")
                x_error = self.central_point_of_camera[0] - flower_with_tag['CentralPoint'][0]
                y_error = self.central_point_of_camera[1] - flower_with_tag['CentralPoint'][1]
                area_error = self.area_of_polliating - flower_with_tag['Area']
                break
        self.angles_of_joints.data = []
        print(x_error)
        print(area_error)
        if abs(x_error) > self.threthold_of_x_error:
           # print("关节速度为", self.joint_speed)
            self.arm_params['joint1'] = int(self.limit_num(self.arm_params['joint1'] + copysign(self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
            #print("角度值为：", self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        if abs(area_error) > self.threthold_of_area_error:
            self.arm_params['joint2'] = int(self.limit_num(self.arm_params['joint2'] + copysign(self.joint_speed, area_error), self.default_arm_params['joint2_limiting']))
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        
       # if abs(y_error) > self.threthold_of_y_error:
           # self.arm_params['joint3'] = int(self.limit_num(self.arm_params['joint3'] + copysign(self.joint_speed, y_error), self.default_arm_params['joint3_limiting']))
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        # if abs(y_error) > self.threthold_of_y_error:
            # self.arm_params['joint4'] = int(self.limit_num(self.arm_params['joint4'] + copysign(self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        if (abs(x_error) < self.threthold_of_x_error and
            abs(area_error) < self.threthold_of_area_error): # and
            # abs(y_error) < self.threthold_of_y_error):
            print("已经完成授粉")
            for index, flower_with_tag in enumerate(self.flowers_with_tag):
                if flower_with_tag['Moving'] == True:
                    self.flowers_with_tag_again[index]['Moving'] = False
                    self.flowers_with_tag_again[index]['Pollinated'] = True
            self.reset_arm_pose()
            return
        self.angles_of_joints.data.append(self.servo_time)
        print("发送命令给机械臂")
        print(self.angles_of_joints)
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        # ros2 topic pub /joint_angles "data: [68, 80, 65, 110]"

    def limit_num(self, num, num_range):
        if num > num_range[1]:
            num = num_range[1]
        elif num < num_range[0]:
            num = num_range[0]
        return num

    def reset_arm_pose(self):
        """ 控制 arm 回到初始姿态 """
        print("控制 arm 回到初始姿态")
        self.open_vision_detect = False
        self.pre_process = False
        self.choose_arm_goal("a_left")

    def voice(self, flowers_lists):
        sorted_data = sorted(flowers_lists, key=lambda x: x['CentralPoint'][1])
        goal_list = []
        # 创建目标字典，并按排序后的结果填充值
        for i in range(len(sorted_data)):
            goal_list.append(sorted_data[i]['Type'])
        #语音播报'A'
        if self.place_name == 'A':
            for index, goal in enumerate(goal_list):
                if index == 0:
                    self.voice_broadcast('up')
                    if goal == 'famale':
                        self.voice_broadcast(type='female')
                    else:
                        self.voice_broadcast(type="male")
                if index == 1:
                    self.voice_broadcast('middle')
                    if goal == 'famale':
                        self.voice_broadcast(type='female')
                    else:
                        self.voice_broadcast(type="male")
                if index == 2:
                    self.voice_broadcast('down')
                    if goal == 'famale':
                        self.voice_broadcast(type='female')
                    else:
                        self.voice_broadcast(type="male")
        self.open_vision_detect = False
        #语音播报"B"
        if self.place_name == 'B':
            for index, goal in enumerate(goal_list):
                if index == 0:
                    if goal == 'famale':
                        self.voice_broadcast(type='female')
                    else:
                        self.voice_broadcast(type="male")
    # 废弃
    def arm_move(self, flowers_lists):
            # A 区识别
            if self.place_name == "A":
                sorted_data = sorted(flowers_lists, key=lambda x: x['CentralPoint'][1])
                goal_list = []
                # 创建目标字典，并按排序后的结果填充值
                for i in range(len(sorted_data)):
                    goal_list.append(sorted_data[i]['Type'])

                # print(goal_list)
                # male_num = 0
                # female_num = 0
                # for index, flower in enumerate(flowers_lists):
                #     if flower['Type'] == 'male':
                #         male_num += 1
                #     elif flower['Type'] == 'famale':
                #         female_num += 1
                # 语音播报
                # self.voice_broadcast(male_num, female_num)
                
                for index, goal in enumerate(goal_list):
                    if index == 0:
                        self.voice_broadcast('up')
                        if goal == 'famale':
                            self.voice_broadcast(type='female')
                            self.choose_arm_goal_in_task('middle')
                            self.choose_arm_goal_in_task('a_1')
                        else:
                            self.voice_broadcast(type="male")
                        self.choose_arm_goal_in_task(self.pose_name)
                    if index == 1:
                        self.voice_broadcast('middle')
                        if goal == 'famale':
                            self.voice_broadcast(type='female')
                            self.choose_arm_goal_in_task('middle')
                            self.choose_arm_goal_in_task('a_2')
                        else:
                            self.voice_broadcast(type="male")
                        self.choose_arm_goal_in_task(self.pose_name)
                    if index == 2:
                        self.voice_broadcast('down')
                        if goal == 'famale':
                            self.voice_broadcast(type='female')
                            self.choose_arm_goal_in_task('middle')
                            self.choose_arm_goal_in_task('a_3')
                        else:
                            self.voice_broadcast(type="male")
                        self.choose_arm_goal_in_task(self.pose_name)
                # 授粉完关闭视觉检测
                self.open_vision_detect = False
            elif self.place_name == "B":
                for flower in flowers_lists:
                    if self.pose_name == 'front':
                        if self.run_times == 0:
                            self.voice_broadcast('middle')
                            if flower['Type'] == 'famale':
                                self.voice_broadcast(type="female")
                                self.choose_arm_goal_in_task("b_middle_front")
                            else:
                                self.voice_broadcast(type="male")
                            self.run_times += 1
                            self.open_vision_detect = False
                            return
                        if self.run_times == 1:
                            self.voice_broadcast('left')
                            if flower['Type'] == 'famale':
                                self.voice_broadcast(type="female")
                                self.choose_arm_goal_in_task("b_right_front")
                            else:
                                self.voice_broadcast(type="male")
                            self.run_times += 1
                            self.open_vision_detect = False
                            return
                        if self.run_times == 2:
                            self.voice_broadcast('right')
                            if flower['Type'] == 'famale':
                                self.voice_broadcast(type="female")
                                self.choose_arm_goal_in_task("b_left_front")
                            else:
                                self.voice_broadcast(type="male")
                            self.run_times = 0
                            return
                    if self.pose_name == 'back':
                        if self.run_times == 0:
                            self.voice_broadcast('middle')
                            if flower['Type'] == 'famale':
                                self.voice_broadcast(type="female")
                                self.choose_arm_goal_in_task("b_left_front")
                            else:
                                self.voice_broadcast(type="male")
                            self.run_times += 1
                            self.open_vision_detect = False
                            return
                        if self.run_times == 1:
                            self.voice_broadcast('left')
                            if flower['Type'] == 'famale':
                                self.voice_broadcast(type="female")
                                self.choose_arm_goal_in_task("b_left_front")
                            else:
                                self.voice_broadcast(type="male")
                            self.run_times += 1
                            self.open_vision_detect = False
                            return
                        if self.run_times == 2:
                            self.voice_broadcast('right')
                            if flower['Type'] == 'famale':
                                self.voice_broadcast(type="female")
                                self.choose_arm_goal_in_task("b_right_front")
                            else:
                                self.voice_broadcast(type="male")
                            self.run_times = 0
                            return
                    break
                
    # 废弃
    def choose_arm_goal_in_task(self, pose_name):
        self.arm_params['joint2'] = self.default_arm_params['joint2_'+pose_name]
        self.arm_params['joint3'] = self.default_arm_params['joint3_'+pose_name]
        self.arm_params['joint4'] = self.default_arm_params['joint4_'+pose_name]
        self.angles_of_joints.data = []
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        time.sleep(1.5)

    def choose_arm_goal(self, pose_name):
        self.arm_params['joint1'] = self.default_arm_params['joint1_'+pose_name]
        self.arm_params['joint2'] = self.default_arm_params['joint2_'+pose_name]
        self.arm_params['joint3'] = self.default_arm_params['joint3_'+pose_name]
        self.arm_params['joint4'] = self.default_arm_params['joint4_'+pose_name]
        self.angles_of_joints.data = []
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.angles_of_joints.data.append(self.servo_reset_time)
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        time.sleep(3.0)

    def choose_arm_goal_in_number(self, joint1=0, joint2=0, joint3=0, joint4=0):
        if joint1 != 0:
            self.arm_params['joint1'] = joint1
        if joint2 != 0:
            self.arm_params['joint1'] = joint2
        if joint3 != 0:
            self.arm_params['joint1'] = joint3
        if joint4 != 0:
            self.arm_params['joint1'] = joint4
        self.angles_of_joints.data = []
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        time.sleep(2.0)

    
    def voice_broadcast(self, direction='', type=''):
        """ 语音播报 """
        if direction != '':
            if direction == 'up':
                print("上边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/up.wav', '-D', '0', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'middle':
                print("中间的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/middle.wav', '-D', '0', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'down':
                print("下边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/down.wav', '-D', '0', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'left':
                print("左边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/left.wav', '-D', '0', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'right':
                print("右边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/right.wav', '-D', '0', '-d', '0'])
                time.sleep(2.0)
        if type != '':
            if type == 'male':
                print("雄花")
                subprocess.Popen(['sudo', 'tinyplay', './voice/male.wav', '-D', '0', '-d', '0'])
                time.sleep(1.0)
            elif type == 'female':
                print("雌花")
                subprocess.Popen(['sudo', 'tinyplay', './voice/female.wav', '-D', '0', '-d', '0'])
                time.sleep(1.0)

    def calculate_O_distance(self, point1, point2):
        """ 计算两个坐标点之间的 O 式距离 """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def load_config_file_(self):
        """ 读取 YAML 文件 """
        with open(self.file_path, 'r') as file:
            self.default_arm_params = yaml.safe_load(file)

    def spin_task_(self):
        rclpy.spin(self)

    def lidar_callback_(self, msg):
        """ 单线激光雷达的回调函数 """
        # print("激光")
        self.lidar_distance = msg.range

    def timer_work_(self):
        # 更新参数
        # self.get_param_()
        # ref = self.get_odom_angle_()
        # 姿态控制
        # self.ori_angle_pid.pid_calculate(ref=ref+self.deviation_angle, goal=self.angle)
        # self.move_cmd.angular.z = self.ori_angle_pid.out

        # 距离控制
        if self.start_for_pid_distance:
            self.position = self.get_coordinate_value_()

            o_distance = self.get_O_distance()
            o_distance *= self.odom_linear_scale_correction # 修正
            # print("在上一次停下后已经行驶的距离: ", o_distance)

            # 计算误差
            self.distance_error = o_distance - abs(self.distance) # 负值控制车向前，正值控制车向后
            # print("误差当前值为: ", self.distance_error)

            self.distance_pid.pid_calculate(o_distance, abs(self.distance))
            if self.distance >= 0:
                self.move_cmd.linear.x = self.distance_pid.out
            else:
                # print("distance 为负值")
                self.move_cmd.linear.x = -self.distance_pid.out
            print(self.move_cmd.linear.x)
            if abs(self.distance_error) < self.distance_tolerance: # 达到目标的情况
                self.start_for_pid_distance = False
                # print("任务已完成......")
        elif self.start_for_lidar_distance:
            self.move_cmd.linear.x = self.liear_speed
        else: # 未设定目标的情况
            self.move_cmd.linear.x = 0.0
            self.x_start = self.get_position_().transform.translation.x
            self.y_start = self.get_position_().transform.translation.y
            # print(self.move_cmd)
            # print("正在停车状态......")
        self.cmd_vel.publish(self.move_cmd)

    def get_coordinate_value_(self):
        position = Point()
        position.x = self.get_position_().transform.translation.x
        position.y = self.get_position_().transform.translation.y
        return position
     
    def get_position_(self):
        try:
            now = rclpy.time.Time()
            # 从 self.tf_buffer 中查询从 self.odom_frame 到 self.base_frame 之间的坐标变换信息，并且在当前时间 now 进行查询
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, now, Duration(seconds=5))   
            return trans       
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready.')
            raise
            return
         
    def get_O_distance(self):
        return sqrt(pow((self.position.x - self.x_start), 2) + pow((self.position.y - self.y_start), 2))

    # 废弃
    def get_odom_angle_(self):
        """ 得到目前的子坐标系相对夫坐标系转动的角度(弧度制)，父坐标系是不动的 """
        try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, now, Duration(seconds=5))
            # 创建了一个四元数对象   
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            """ 获取旋转矩阵的欧拉角。GetRPY()返回的是一个长度为3的列表, 包含了旋转矩阵的roll、pitch和yaw角度。
                    在这里, [2]索引表示取得yaw角度, 也就是绕z轴的旋转角度 """
            angle_rot = cacl_rot.GetRPY()[2]
            return angle_rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return   

def main():
    rclpy.init()
    try:
        node = Game_Controller("Game_Controller")
        node.choose_arm_goal("a_left")
        node.start_car_and_lidar_controls_stopping(0.05, 0.4)
        node.vision_control_arm("A","a_left")
        node.vision_control_arm("A","a_right")
        print("第一个点位已授粉完成")
        node.start_car_and_lidar_controls_stopping(0.05, 0.4)
        node.vision_control_arm("A","a_right")
        node.vision_control_arm("A","a_left")
        print("第二个点位已授粉完成")
        node.start_car_and_lidar_controls_stopping(0.05, 0.4)
        node.vision_control_arm("A","a_left")
        node.vision_control_arm("A","a_right")
        print("第三个点位已授粉完成")
        #node.find_next_arm_goal_on_position()
        while 1:
            pass
    except KeyboardInterrupt:
        print("退出暂停小车！！！！！！！！！")
        node.cmd_vel.publish(Twist())
        node.cmd_vel.publish(Twist())
        node.cmd_vel.publish(Twist())
        node.cmd_vel.publish(Twist())
        node.cmd_vel.publish(Twist())
        node.cmd_vel.publish(Twist())
        time.sleep(2.0)
    finally:
        if node:
            node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()

