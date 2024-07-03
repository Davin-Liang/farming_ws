#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets # type: ignore
import os
import yaml
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import time
import subprocess
import copy
from threading import Thread
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range
from math import copysign, sqrt, pow, radians
from pid import PID

class Game_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        # 加载 arm 参数
        self.file_path = os.path.expanduser('~/farming_ws/src/motion_controller/config/arm_params.yaml')
        self.load_config_file_()
    
        # 重要 BOOL 值
        self.open_vision_detect         = False # 是否打开视觉检测
        self.pre_process                = False # 是否打开数据预处理
        self.start_for_lidar_distance   = False
        self.start_for_pid_distance     = False
        self.voice_switch               = False
        self.start_count                = False
        self.error                      = False
        self.only_arm_action            = False
        self.one_action                 = False

        # 数据字典
        self.arm_params = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0} # 存储实时的机械臂角度
        self.flowers_with_tag = [] # 存储花属性
        self.flowers_with_tag_again = [] # 存储花属性
        self.flowers_lists = [] # primitive flower data
        self.last_flowers_lists = []

        # 可调参数
        self.area_scaling_factor                     = 0.25 # 面积缩放系数
        self.O_distance_threthold_of_judge_same_goal = 104 # 判断前后两次数据检测的识别框是否为同一个目标的阈值
        self.central_point_of_camera                 = [320, 240] # 相机中心点
        self.area_of_polliating                      = 80000 # 识别框为多少时才进行授粉的面积阈值 70000
        self.joint_speed                             = 1.0 # 关节转动速度，将关节的转动的角度当作速度
        self.threthold_of_x_error                    = 15.0
        self.threthold_of_y_error                    = 15.0
        self.threthold_of_area_error                 = 7000.0
        self.servo_time                              = 140  #机械臂运动时间，单位mm
        self.servo_reset_time                        = 2000  #机械臂初始位置运动时间
        self.distance_tolerance                      = 0.03
        self.angle_tolerance                         = radians(2.0)
        self.odom_linear_scale_correction            = 1.0
        self.odom_angular_scale_correction           = 1.0
        self.area_difference                         = 30000    #TODO: 修改阈值
        self.time_threshold                          = 1.0     #时间阈值

        # 使用到的订阅者和发布者
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Int32MultiArray, "servo_commands", 10)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.buzzer_publisher_ = self.create_publisher(Bool, "/Buzzer", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "laser", self.lidar_callback_, 10)
        self.odom_subcriber_ = self.create_subscription(Odometry, "odom", self.odom_callback_, 10)
        self.yaw_angle_subcriber_ = self.create_subscription(Float64, "yaw_angle", self.yaw_angle_callback_, 10)

        self.move_cmd           = Twist()
        self.angles_of_joints   = Int32MultiArray()
        self.buzzer_cmd         = Bool()

        self.ori_angle_pid = PID(Kp=0.685, Ki=0.00079, Kd=0.426, max_out=0.9, max_iout=0.0085)
        self.distance_pid  = PID(Kp=0.42, Ki=0.0, Kd=0.08, max_out=0.85, max_iout=0.0)

        self.female_num      = 0
        self.distance        = 0.0
        self.angle           = 0.0
        self.yaw_angle       = 0.0 # testing
        self.lidar_threthold = 0.1
        self.liear_speed     = 0.5
        self.angle = radians(self.angle)
        self.deviation_angle = radians(0.85)

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y

        self.distance_error = 0
        self.angle_error    = 0

        time.sleep(2.0)

        # 创建定时器
        self.work_timer = self.create_timer(0.004, self.timer_work_)
        self.arm_timer = self.create_timer(0.18, self.arm_timer_callback_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

# -----------------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------对外接口函数------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------
    def auto_pollinate(self, place="A", arm_pose="a_left"):
        if "A" == place or "C" == place:
            self.vision_control_arm(place, arm_pose)
            if self.only_arm_action:
                return
            if self.one_action:
                return
            for i in range(self.female_num-1):
                print(self.female_num)
                self.find_next_arm_goal_on_position()
                if self.error:
                    print('重新给次机会')
                    self.find_next_arm_goal_on_position()
                    self.error = False
        elif "B" == place:
            self.vision_control_arm(place, arm_pose)
            if self.error == True:
                self.find_next_arm_goal_on_position()
    
    def buzzer_tips(self, times=1.0):
        pass
        self.buzzer_cmd.data = True
        self.buzzer_publisher_.publish(self.buzzer_cmd)
        time.sleep(times)
        self.buzzer_cmd.data = False
        self.buzzer_publisher_.publish(self.buzzer_cmd)

    def vision_control_arm(self, place_name, pose_name):
        self.place_name = place_name
        self.choose_arm_goal(pose_name)
        if self.only_arm_action:
            return
        self.open_vision_detect = True
        self.pre_process        = True
        self.arm_moving         = False
        self.reset_vision_data()
        # 堵塞函数直到完成任务
        # print("正在等待完成任务......")
        while self.open_vision_detect:
            pass
        print("已经完成任务！！！！！！")

    def reset_vision_data(self):
        self.flowers_with_tag.clear()
        self.flowers_with_tag_again.clear()
        self.female_num = 0

    def find_next_arm_goal_on_position(self):
        self.open_vision_detect = True
        self.pre_process = True
        # 堵塞函数直到完成任务
        # print("正在等待完成任务......")
        while self.open_vision_detect:
            pass
        print("已经完成任务！！！！！！")

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
        while abs(self.angle-self.yaw_angle)>self.angle_tolerance:
            pass
        time.sleep(4.0)

    def car_action_in_lidar(self, speed, threthold=0.1, mode=1):
        """ 开动车并使用单线激光控制小车停止 """
        self.liear_speed = speed
        self.lidar_threthold = threthold
        self.start_for_lidar_distance = True

        # 等待 car 到位
        time.sleep(2.0) # 保证 car 驶出激光遮挡区域
        # print("激光未受到目标的遮挡......")
        if mode == 1:
            while self.lidar_distance > self.lidar_threthold:
                pass
        elif mode == 0:
            while self.lidar_distance > self.lidar_threthold:
                pass
        print("已经到达激光遮挡区域!!!")
        self.start_for_lidar_distance = False
        time.sleep(2.0)

    def choose_arm_goal(self, pose_name):
        """ Use arm goals in YAML file. """
        self.pose_name = pose_name
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
        """ Use own numbers to control arm instead of arm goals in YAML file. """
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

# -----------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------END------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------

    def confrim_moving_goal_for_arm_(self, flowers_lists):
        """ 确定 arm 的移动目标 """
        # 数据预处理并选择第一个处理的目标
        self.data_pre_processing_(flowers_lists)
        # 更新数据
        for flower in flowers_lists:
            print("有数据")
            if flower['Type'] == "famale":
                print("满足条件 0")
                for index, flower_with_tag in enumerate(self.flowers_with_tag):
                    if flower_with_tag['Moving'] == True:
                        if self.calculate_O_distance_(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                            print("满足条件 1")
                            if abs(flower_with_tag['Area']-flower['Area']) <= self.area_difference:
                                print("满足条件 2")
                                self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                                self.flowers_with_tag[index]['Area'] = flower['Area']
                                self.control_arm_()
                                break # 跳出内层 for 循环

        # # 控制 arm
        # self.control_arm_()

    def data_pre_processing_(self, flowers_lists):
        """ 为存储花属性的字典添加花属性：是否正在操作、是否已授粉 """
        # 类型、中心的坐标、面积、是否正在操作、是否已授粉
        flower_with_tag = {'Type': '', 'CentralPoint': [], 'Area': 0, 'Moving': False, 'Pollinated': False}
        if self.pre_process:
            if len(self.flowers_with_tag) == 0:
                print("正在进行数据预处理")
                for index, flower in enumerate(flowers_lists):
                    self.female_num += 1
                    if self.female_num >= 3:
                        self.female_num = 3
                    if flower['Type'] == 'famale':
                        flower_with_tag['Type'] = flower['Type']
                        flower_with_tag['CentralPoint'] = flower['CentralPoint']
                        flower_with_tag['Area'] = flower['Area']
                        if self.arm_moving == False:
                            flower_with_tag['Moving'] = True
                            self.arm_moving = True
                        else:
                            flower_with_tag['Moving'] = False
                        self.flowers_with_tag.append(copy.deepcopy(flower_with_tag))
                self.flowers_with_tag_again = copy.deepcopy(self.flowers_with_tag)
                #添加语音播报
                if self.voice_switch:
                    self.voice_(flowers_lists)
            else:
                print("正在授粉下一个目标点")
                self.flowers_with_tag = copy.deepcopy(self.flowers_with_tag_again)
                print("预处理第二次")
                # 更新中心点坐标参数
                for flower in flowers_lists:
                    for index, flower_with_tag in enumerate(self.flowers_with_tag):
                        if self.calculate_O_distance_(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                            self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                            self.flowers_with_tag[index]['Area'] = flower['Area']
                            break
                # 寻找未“授粉”的花，找到的第一朵就设置为目标
                for index, flower_with_tag in enumerate(self.flowers_with_tag):
                    if flower_with_tag['Pollinated'] != True:
                        self.flowers_with_tag[index]['Moving'] = True
                        break
        self.pre_process = False # 关闭数据预处理

    def control_arm_(self):
        y_error = 0
        x_error = 0
        area_error = 0
        # print(self.flowers_with_tag)
        for flower_with_tag in self.flowers_with_tag:
            if flower_with_tag['Moving'] == True:
                # print("正在获取 x 轴误差")
                x_error = self.central_point_of_camera[0] - flower_with_tag['CentralPoint'][0]
                y_error = self.central_point_of_camera[1] - flower_with_tag['CentralPoint'][1]
                area_error = self.area_of_polliating - flower_with_tag['Area']
                break
        self.angles_of_joints.data = []
        # print(x_error)
        # print(area_error)
        if abs(x_error) > self.threthold_of_x_error:
            self.arm_params['joint1'] = int(self.limit_num_(self.arm_params['joint1'] + copysign(self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        if abs(area_error) > self.threthold_of_area_error:
            self.arm_params['joint2'] = int(self.limit_num_(self.arm_params['joint2'] + copysign(self.joint_speed, area_error), self.default_arm_params['joint2_limiting']))
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        if abs(y_error) > self.threthold_of_y_error:
            self.arm_params['joint4'] = int(self.limit_num_(self.arm_params['joint4'] + copysign(self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        if (abs(x_error) < self.threthold_of_x_error and
            abs(area_error) < self.threthold_of_area_error): # and
            # abs(y_error) < self.threthold_of_y_error):
            print("已经完成授粉")
            for index, flower_with_tag in enumerate(self.flowers_with_tag):
                if flower_with_tag['Moving'] == True:
                    self.flowers_with_tag_again[index]['Moving'] = False
                    self.flowers_with_tag_again[index]['Pollinated'] = True
            self.reset_arm_pose_(self.pose_name)
            return
        self.angles_of_joints.data.append(self.servo_time)
        # print("发送命令给机械臂")
        # print(self.angles_of_joints)
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        # ros2 topic pub /joint_angles "data: [68, 80, 65, 110]"

    def limit_num_(self, num, num_range):
        if num > num_range[1]:
            num = num_range[1]
        elif num < num_range[0]:
            num = num_range[0]
        return num

    def reset_arm_pose_(self, pose="a_left"):
        """ 控制 arm 回到初始姿态 """
        print("控制 arm 回到初始姿态")
        self.choose_arm_goal(pose)
        self.open_vision_detect = False
        self.pre_process = False
        time.sleep(2.0)

    def voice_(self, flowers_lists):
        #语音播报'A'
        if self.place_name == 'A':
            goal_list = self.data_sort_(flowers_lists, prior_axis='v')
                
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
            return
        #self.open_vision_detect = False
        #语音播报"B"
        if self.place_name == 'B':
            goal_list = self.data_sort_(flowers_lists, prior_axis='v')

            for index, goal in enumerate(goal_list):
                if index == 0:
                    if goal == 'famale':
                        self.voice_broadcast(type='female')
                    else:
                        self.voice_broadcast(type="male")
            return
        if self.place_name == "C":
            goal_list = self.data_sort_(flowers_lists, prior_axis='h')

            for index, goal in enumerate(goal_list):
                if index == 0:
                    self.voice_broadcast('left')
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
                    self.voice_broadcast('right')
                    if goal == 'famale':
                        self.voice_broadcast(type='female')
                    else:
                        self.voice_broadcast(type="male")
            return

    def data_sort_(self, flowers_lists, prior_axis='h'):
        """ 'h' represents horizontal, 'v' represents vertical. """
        goal_list = []
        if prior_axis == 'h':
            sorted_data = sorted(flowers_lists, key=lambda x: x['CentralPoint'][0])
            for i in range(len(sorted_data)):
                goal_list.append(sorted_data[i]['Type'])
        elif prior_axis == 'v':
            sorted_data = sorted(flowers_lists, key=lambda x: x['CentralPoint'][1])
            for i in range(len(sorted_data)):
                goal_list.append(sorted_data[i]['Type'])

        return goal_list

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

    def calculate_O_distance_(self, point1, point2):
        """ 计算两个坐标点之间的 O 式距离 """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def load_config_file_(self):
        """ 读取 YAML 文件 """
        with open(self.file_path, 'r') as file:
            self.default_arm_params = yaml.safe_load(file)

    def get_O_distance_(self):
        return sqrt(pow((self.position.x - self.x_start), 2) + pow((self.position.y - self.y_start), 2))


    def yaw_angle_callback_(self, msg):
        self.yaw_angle = -msg.data

    def odom_callback_(self, msg):
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y

    def arm_timer_callback_(self):
        # print("正在等待开启视觉")
        if not self.open_vision_detect:
            return

        if self.last_flowers_lists == self.flowers_lists or len(self.flowers_lists) > 5:
            # if self.start_count == False:
            #     self.start_count = True
            #     self.start_count_time = time.time()
            # if time.time() - self.start_count_time > self.time_threshold:
            #     self.start_count = False
            #     self.error = True
            #     print('目标点丢失')
            #     self.reset_arm_pose_(self.pose_name)
            pass
                
        else:
            if 0 != len(self.flowers_lists): # 预防处理空数据
                self.start_count = False
                # print(self.flowers_lists)
                self.confrim_moving_goal_for_arm_(self.flowers_lists) 
                self.last_flowers_lists = self.flowers_lists

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

    def spin_task_(self):
        rclpy.spin(self)

    def lidar_callback_(self, msg):
        """ 单线激光雷达的回调函数 """
        # print("激光")
        self.lidar_distance = msg.range

    def timer_work_(self):
        # 姿态控制
        self.ori_angle_pid.pid_calculate(ref=self.yaw_angle, goal=self.angle)
        self.move_cmd.angular.z = self.ori_angle_pid.out

        # 距离控制
        if self.start_for_pid_distance:
            o_distance = self.get_O_distance_()
            o_distance *= self.odom_linear_scale_correction # 修正
            # 计算误差
            self.distance_error = o_distance - abs(self.distance) # 负值控制车向前，正值控制车向后
            self.distance_pid.pid_calculate(o_distance, abs(self.distance))
            if self.distance >= 0:
                self.move_cmd.linear.x = self.distance_pid.out
            else:
                self.move_cmd.linear.x = -self.distance_pid.out
            # print(self.move_cmd.linear.x)
            if abs(self.distance_error) < self.distance_tolerance: # 达到目标的情况
                self.start_for_pid_distance = False
        elif self.start_for_lidar_distance:
            self.move_cmd.linear.x = self.liear_speed
        else: # 未设定目标的情况
            self.move_cmd.linear.x = 0.0
            self.x_start = self.position.x
            self.y_start = self.position.y
        self.cmd_vel.publish(self.move_cmd)
        






def main():
    rclpy.init()
    try:
        node = Game_Controller("Game_Controller")

# ---------------------------------------------------------------------------------------------------------------
# ----------------AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA--------------------
# ---------------------------------------------------------------------------------------------------------------
        if node.A_switch:
            for i in range(3):
                node.car_action_in_lidar(0.05, 0.4)
                node.vision_control_arm("A","a_left")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.vision_control_arm("A","a_right")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
            node.set_distance(0.8) # TODO: 距离未确定
            node.set_angle(-90.0)
            node.car_action_in_lidar(-0.05, 0.6)
            node.car_action_in_lidar(-0.05, 0.6)
            node.set_distance(-0.35) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB--------------------
# ---------------------------------------------------------------------------------------------------------------
        if node.B_switch:
            node.car_action_in_lidar(-0.05, 0.4)
            node.set_distance(0.15) #TODO: 距离未确定
            # TODO:机械臂序号未确定
            node.vision_control_arm("B","a_left") 
            node.vision_control_arm("B","a_left")
            node.vision_control_arm("B","a_left")
            node.choose_arm_goal("a_left")
            for i in range(3):
                node.car_action_in_lidar(-0.05, 0.4)
                node.set_distance(-0.15) #TODO: 距离未确定
                # arm action
                # TODO:机械臂序号未确定
                node.vision_control_arm("B","a_left") 
                node.vision_control_arm("B","a_left")
                node.vision_control_arm("B","a_left")
                
                if i == 2:
                    break
                # TODO:机械臂序号未确定 
                node.vision_control_arm("B","a_left") 
                node.vision_control_arm("B","a_left")
                node.vision_control_arm("B","a_left")
                node.choose_arm_goal("a_left")

            node.set_distance(-0.2) #TODO: 距离未确定
            node.set_angle(90.0)
            node.car_action_in_lidar(0.05, 0.6)
            node.car_action_in_lidar(0.05, 0.6)
            node.set_distance(0.25) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC--------------------
# ---------------------------------------------------------------------------------------------------------------
        if node.C_switch:
            for i in range(3):
                node.car_action_in_lidar(0.05, 0.4)
                node.set_distance(0.1) #TODO: 距离未确定
                node.vision_control_arm("C","a_left")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.vision_control_arm("C","a_right")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.car_action_in_lidar(0.05, 0.4, 0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH--------------------
# ---------------------------------------------------------------------------------------------------------------
        if node.Home_switch:
            pass
# ---------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------

        while 1:
            pass
    except KeyboardInterrupt:
        print("退出暂停小车！！！！！！！！！")
        node.cmd_vel.publish(Twist())
        node.cmd_vel.publish(Twist())
        time.sleep(2.0)
    finally:
        if node:
            node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()

