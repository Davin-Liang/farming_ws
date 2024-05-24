#!/usr/bin/env python3
import math
import rclpy
import cv2, cv_bridge
import numpy as np
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets # type: ignore
import os
import yaml
from std_msgs.msg import Int16MultiArray
import time
import subprocess
import copy
from threading import Thread
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range, Image
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import sqrt, pow, radians
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import PyKDL
from pid import PID

class Game_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        # 加载 arm 参数
        self.file_path = os.path.expanduser('~/farming_ws/src/farming_vision/config/arm_params.yaml')
        self.load_config_file_()

        self.bridge = cv_bridge.CvBridge()
    
        # 重要 BOOL 值
        self.open_vision_detect = False # 是否打开视觉检测
        self.open_vision_patrol = False # 是否打开视觉寻线
        # 数据字典
        self.arm_params = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0} # 存储实时的机械臂角度
        # 使用到的订阅者和发布者
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Int16MultiArray, "joint_angles", 100)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "laser", self.lidar_callback_, 10)
        self.angles_of_joints = Int16MultiArray()

        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(Image, '/camera/process_image', 10)

        self.move_cmd = Twist()

        self.ori_angle_pid = PID(0.685, 0.0, 0.426, 1.4, 0.0)
        self.distance_pid = PID(0.42, 0.0, 0.08, 1.0, 0.0)

        
        self.distance = 0.0
        self.angle = 0.0
        self.angle = radians(self.angle)
        self.deviation_angle = radians(0.65)
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
        
        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y

        self.distance_error = 0
        self.angle_error    = 0
        self.run_times = 0

        time.sleep(5.0)

        # 创建定时器
        self.work_timer = self.create_timer(0.04, self.timer_work_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

    def image_callback(self, msg):
        # print("正在进行巡线图像处理")
        # np_arr = np.fromstring(msg.data, np.uint8)
        np_arr = np.array(msg.data, dtype=np.uint8)
        # 使用 OpenCV 解码 JPEG 数据
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 定义白色的HSV颜色范围
        # 白色的HSV颜色范围的下界和上界
        # 色调值从0到180（覆盖所有色调），饱和度值从0到25（低饱和度，即接近灰色），明度值从200到255（高亮度）
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        # 创建一个掩模，表示图像中白色部分的区域
        # # 检查图像中的每个像素，如果像素值在 lower_white 和 upper_white 范围内，则该像素在掩模中对应的位置会被设置为255（白色），否则会被设置为0（黑色）。
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 获取图像的高度、宽度和深度
        h, w, d = image.shape
        # 定义搜索区域的上下边界
        search_top = int(h / 2)
        search_bot = int(h / 2 + 20)
        # 将掩模的上半部分和下半部分设置为0，仅保留中间一条窄带的掩模
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # 计算掩模区域的图像矩（Moments），用于后续的质心计算
        M = cv2.moments(mask)

        if M['m00'] > 0:
            # 计算白色区域的质心
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # print("cx = ", cx)
            # print("cy = ", cy)
            # 在图像中绘制一个红色圆圈，标记出质心位置
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            # 基于检测的目标中心点，计算机器人的控制参数
            err = cx - w / 2
            if self.open_vision_patrol:
                self.move_cmd.angular.z = float(err) / 1000
                if abs(self.move_cmd.angular.z) > 0.8:
                    self.move_cmd.angular.z = math.copysign(1.4, self.move_cmd.angular.z)
                print(self.move_cmd.angular.z)

        # 发布处理后的图像
        self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

    # ---------------- 对外接口函数 -----------------
    def set_vision_patrol_mode(self, mode):
        if mode == 1:
            self.open_vision_patrol = True
        elif mode == 0:
            self.open_vision_patrol = False
            self.move_cmd.angular.z  = 0.0

    def control_car_turn(self, turn_speed, turn_time):
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
        time.sleep(2.0) # 保证 car 驶出激光遮挡区域
        for i in range(ignore_num+1):
            print("正在检测中")
            print("激光", self.lidar_distance)
            print("阈值", self.lidar_threthold)
            while self.lidar_distance > self.lidar_threthold:
                pass
        print("激光已经到达下一个激光遮挡区域")
        self.start_for_lidar_distance = False
        time.sleep(2.0)

    def vision_choose_goal_in_A(self, pose_name):
        """ 传入视觉目标 """
        self.place_name = "A"
        self.pose_name = pose_name
        self.choose_arm_goal_alone(pose_name)
        self.open_vision_detect     = True
        # 堵塞函数直到完成任务
        # print("正在等待完成任务")
        while self.open_vision_detect:
            pass

    def vision_choose_goal_in_B(self, pose_name):
        self.place_name = "B"
        self.pose_name = pose_name
        self.run_times = 0
        if pose_name == "front":
            # 中间花
            self.choose_arm_goal_alone("b_middle_front_pre")
            while self.open_vision_detect:
                pass
            
            self.choose_arm_goal_alone("b_middle_front_pre")
            self.choose_arm_goal_in_number(joint1=126)
            self.choose_arm_goal_alone("b_right_front_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_alone("b_right_front_pre")
            self.choose_arm_goal_alone("b_left_front_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_alone("b_left_front_pre")
            self.choose_arm_goal_in_number(joint2=107, joint3=124, joint4=93)
            self.choose_arm_goal_alone("b_middle_front_pre")
        if pose_name == "back":
            self.choose_arm_goal_alone("b_middle_back_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_alone("b_middle_back_pre")
            self.choose_arm_goal_in_number(joint1=97)
            self.choose_arm_goal_alone("b_left_back_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_alone("b_left_back_pre")
            self.choose_arm_goal_alone("b_right_back_pre")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_alone("b_right_back_pre")
            self.choose_arm_goal_in_number(joint2=107, joint3=124, joint4=93)
            self.choose_arm_goal_alone("b_middle_back_pre")

        

    def vision_callback_(self, msg):
        """ 视觉回调函数 """
        # print("正在等待开启视觉")
        if not self.open_vision_detect:
            return
        # print("正在通过视觉控制机械臂")

        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0} # 类型、中心点坐标、面积
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                # print(msg.targets[i].type)
                # print(msg.targets[i].rois[0])
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

        if 0 != len(flowers_lists): # 预防处理空数据
            # print(flowers_lists)
            self.confrim_moving_goal_for_arm(flowers_lists)

    def confrim_moving_goal_for_arm(self, flowers_lists):
        """ 确定 arm 的移动目标 """
        self.arm_move(flowers_lists)

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
                            self.choose_arm_goal_in_task('down')
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

    def choose_arm_goal_alone(self, pose_name):
        self.arm_params['joint1'] = self.default_arm_params['joint1_'+pose_name]
        self.arm_params['joint2'] = self.default_arm_params['joint2_'+pose_name]
        self.arm_params['joint3'] = self.default_arm_params['joint3_'+pose_name]
        self.arm_params['joint4'] = self.default_arm_params['joint4_'+pose_name]
        self.angles_of_joints.data = []
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        time.sleep(6.0)

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
                subprocess.Popen(['sudo', 'tinyplay', './voice/up.wav', '-D', '1', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'middle':
                print("中间的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/middle.wav', '-D', '1', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'down':
                print("下边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/down.wav', '-D', '1', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'left':
                print("左边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/left.wav', '-D', '1', '-d', '0'])
                time.sleep(2.0)
            elif direction == 'right':
                print("右边的花为")
                subprocess.Popen(['sudo', 'tinyplay', './voice/right.wav', '-D', '1', '-d', '0'])
                time.sleep(2.0)
        if type != '':
            if type == 'male':
                print("雄花")
                subprocess.Popen(['sudo', 'tinyplay', './voice/male.wav', '-D', '1', '-d', '0'])
                time.sleep(1.0)
            elif type == 'female':
                print("雌花")
                subprocess.Popen(['sudo', 'tinyplay', './voice/female.wav', '-D', '1', '-d', '0'])
                time.sleep(1.0)
        # if male_num == 0 and female_num == 3:
        #     print("雄花数量为 0 朵，雌花为 3 朵")
        #     subprocess.Popen(['sudo', 'tinyplay', './voice/voice_0_3.wav', '-D', '1', '-d', '0'])
        # elif male_num == 1 and female_num == 2:
        #     print("雄花数量为 1 朵，雌花为 2 朵")
        #     subprocess.Popen(['sudo', 'tinyplay', './voice/voice_1_2.wav', '-D', '1', '-d', '0'])
        # elif male_num == 2 and female_num == 1:
        #     print("雄花数量为 2 朵，雌花为 1 朵")
        #     subprocess.Popen(['sudo', 'tinyplay', './voice/voice_2_1.wav', '-D', '1', '-d', '0'])
        # elif male_num == 3 and female_num == 0:
        #     print("雄花数量为 3 朵，雌花为 0 朵")
        #     subprocess.Popen(['sudo', 'tinyplay', './voice/voice_3_0.wav', '-D', '1', '-d', '0'])
        # elif male_num == 0 and female_num == 0:
        #     if type == "male":
        #         print("该授粉点为雄花，不可授粉")
        #         subprocess.Popen(['sudo', 'tinyplay', './voice/male.wav', '-D', '1', '-d', '0'])
        #     elif type == "female":
        #         print("该授粉点为雌花，可授粉")
        #         subprocess.Popen(['sudo', 'tinyplay', './voice/female.wav', '-D', '1', '-d', '0'])

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

        # node.set_angle(90.0)
        # node.choose_arm_goal_alone("vision_patrol_1")
        # time.sleep(2.0)
        # node.set_vision_patrol_mode(1)
        # node.set_distance(0.5)
        # node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
        # node.set_distance(-0.4)
        # node.vision_choose_goal_in_A("a_left")

        # # A区
        print("开始 A 区")
        for i in range(3):
            node.start_car_and_lidar_controls_stopping(0.04, 0.4)
            node.vision_choose_goal_in_A("a_left")
            node.vision_choose_goal_in_A("a_right")
        print("完成 A 区")
        print("正在前往 B 区")
        node.set_distance(0.5)                
        node.control_car_turn(0.5, 4.835)
        # node.set_angle(-90.0)
        time.sleep(1.0)
        node.start_car_and_lidar_controls_stopping(-0.05, 0.5)
        node.start_car_and_lidar_controls_stopping(-0.05, 0.5)
        node.set_distance(-0.25)
        # node.set_angle(0.0)

        node.control_car_turn(-0.5, 4.835)
        print("开始 B 区")
        node.choose_arm_goal_in_number(joint2=107, joint3=124, joint4=93)
        node.choose_arm_goal_alone("b_middle_front")
        node.start_car_and_lidar_controls_stopping(-0.05, 0.5)
        node.set_distance(0.25)
        node.vision_choose_goal_in_B("front")
        
        for i in range(2):
            node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
            node.set_distance(-0.25)
            node.vision_choose_goal_in_B("front")
            node.vision_choose_goal_in_B("back")

        node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
        node.set_distance(0.1)
        node.vision_choose_goal_in_B("back")


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
    finally:
        if node:
            node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()