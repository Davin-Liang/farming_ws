#!/usr/bin/env python3
import math
import rclpy
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
from sensor_msgs.msg import Range
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
        # 重要 BOOL 值
        self.open_vision_detect = False # 是否打开视觉检测
        # 数据字典
        self.arm_params = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0} # 存储实时的机械臂角度
        # 使用到的订阅者和发布者
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Int16MultiArray, "joint_angles", 100)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "laser", self.lidar_callback_, 10)
        self.angles_of_joints = Int16MultiArray()
        self.move_cmd = Twist()

        self.ori_angle_pid = PID(0.51, 0.0, 0.126, 1.6, 0.0)
        self.distance_pid = PID(0.42, 0.0, 0.08, 1.0, 0.0)

        
        self.distance = 0.0
        self.angle = 0.0
        self.angle = radians(self.angle)
        self.liear_speed = 0.5
        self.distance_tolerance = 0.03
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

    # ---------------- 对外接口函数 -----------------
    def set_distance(self, distance):
        """ 设置车轮方向的行驶距离及以什么样的速度行驶 """
        self.distance = distance
        self.start_for_pid_distance = True
        # 等待完成任务
        while not self.start_for_pid_distance:
            pass

    def set_angle(self, angle):
        """ 设置底盘转动角度 """
        self.angle = angle
        # 等待转完角度
        pass

    def start_car_and_lidar_controls_stopping(self, speed, threthold=0.1, ignore_num=0):
        """ 开动车并使用单线激光控制小车停止 """
        self.liear_speed = speed
        self.lidar_threthold = 0.1
        self.start_for_lidar_distance = True

        # 等待 car 到位
        time.sleep(1.0) # 保证 car 驶出激光遮挡区域
        real_ignore_num = 0
        print(self.lidar_threthold)
        while self.lidar_distance > self.lidar_threthold or real_ignore_num != ignore_num:
            if ignore_num == 0:
                pass
            else:
                real_ignore_num += 1
        print("激光已经到达下一个激光遮挡区域")
        self.start_for_lidar_distance = False

    def vision_choose_goal_in_A(self, pose_name):
        """ 传入视觉目标 """
        self.choose_arm_goal_in_task_alone(pose_name)
        self.open_vision_detect     = True
        # 堵塞函数直到完成任务
        print("正在等待完成任务")
        while self.open_vision_detect:
            pass

    def vision_choose_goal_in_B(self, pose_name):
        self.pose_name = pose_name
        if pose_name == "front":
            self.choose_arm_goal_in_task_alone("a_left_pre_front")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_in_task_alone("a_middle_pre_front")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_in_task_alone("a_right_pre_front")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_in_task_alone("moving")
        if pose_name == "back":
            self.choose_arm_goal_in_task_alone("a_left_pre_back")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_in_task_alone("a_middle_pre_back")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_in_task_alone("a_right_pre_back")
            while self.open_vision_detect:
                pass
            self.choose_arm_goal_in_task_alone("moving")

        

    def vision_callback_(self, msg):
        """ 视觉回调函数 """
        print("正在等待开启视觉")
        if not self.open_vision_detect:
            return
        print("正在通过视觉控制机械臂")

        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0} # 类型、中心点坐标、面积
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                print(msg.targets[i].type)
                print(msg.targets[i].rois[0])
                flower['CentralPoint'].clear()
                if msg.targets[i].type == "male": 
                    flower['Type'] = msg.targets[i].type

                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    flower['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width * self.area_scaling_factor
                elif msg.targets[i].type == "famale":
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.width/2)
                    flower['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width * self.area_scaling_factor

                # 得到原始数据

                flowers_lists.append(copy.deepcopy(flower)) # 深拷贝

        if 0 != len(flowers_lists): # 预防处理空数据
            print(flowers_lists)
            self.confrim_moving_goal_for_arm(flowers_lists)

    def confrim_moving_goal_for_arm(self, flowers_lists):
        """ 确定 arm 的移动目标 """
        self.arm_move(flowers_lists)

    def arm_move(self, flowers_lists):
            if len(flowers_lists) == 3:
                sorted_data = sorted(flowers_lists, key=lambda x: x['CentralPoint'][1])
                # 创建目标字典，并按排序后的结果填充值
                goal_list = [
                    sorted_data[0]['Type'],
                    sorted_data[1]['Type'],
                    sorted_data[2]['Type']
                ]
                print(goal_list)
                male_num = 0
                female_num = 0
                for index, flower in enumerate(flowers_lists):
                    if flower['Type'] == 'male':
                        male_num += 1
                    elif flower['Type'] == 'famale':
                        female_num += 1
                # 语音播报
                self.voice_broadcast(male_num, female_num)
                
                for index, goal in enumerate(goal_list):
                    if index == 0:
                        if goal == 'famale':
                            print("正在去目标1")
                            self.choose_arm_goal_in_task('middle')
                            self.choose_arm_goal_in_task('a_1')
                            self.choose_arm_goal_in_task(self.pose_name)
                    if index == 1:
                        if goal == 'famale':
                            print("正在去目标2")
                            self.choose_arm_goal_in_task('middle')
                            self.choose_arm_goal_in_task('a_2')
                            self.choose_arm_goal_in_task(self.pose_name)
                    if index == 2:
                        if goal == 'famale':
                            print("正在去目标3")
                            self.choose_arm_goal_in_task('a_3')
                            self.choose_arm_goal_in_task(self.pose_name)
                
                self.open_vision_detect = False
            else:
                for flower in flowers_lists:
                    if flower['Type'] == 'famale':
                        if self.pose_name == 'front':
                            if self.run_times == 0:
                                self.choose_arm_goal_in_task("b_left_front")
                                self.run_times += 1
                            if self.run_times == 1:
                                self.choose_arm_goal_in_task("b_middle_front")
                                self.run_times += 1
                            if self.run_times == 2:
                                self.choose_arm_goal_in_task("b_right_front")
                                self.run_times = 0
                        if self.pose_name == 'back':
                            if self.run_times == 0:
                                self.choose_arm_goal_in_task("b_left_back")
                                self.run_times += 1
                            if self.run_times == 1:
                                self.choose_arm_goal_in_task("b_middle_back")
                                self.run_times += 1
                            if self.run_times == 2:
                                self.choose_arm_goal_in_task("b_left_back")
                                self.run_times = 0                        
                    break
                self.open_vision_detect = False

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

    def choose_arm_goal_in_task_alone(self, pose_name):
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
        time.sleep(1.5)
    
    def voice_broadcast(self, male_num=0, female_num=0, type='male'):
        """ 语音播报 """
        if male_num == 0 and female_num == 3:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_0_3.wav', '-D', '0', '-d', '1'])
        elif male_num == 1 and female_num == 2:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_1_2.wav', '-D', '0', '-d', '1'])
        elif male_num == 2 and female_num == 1:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_2_1.wav', '-D', '0', '-d', '1'])
        elif male_num == 3 and female_num == 0:
            subprocess.Popen(['sudo', 'tinyplay', './voice/voice_3_0.wav', '-D', '0', '-d', '1'])
        elif male_num == 0 and female_num == 0:
            if type == "male":
                subprocess.Popen(['sudo', 'tinyplay', './voice/male.wav', '-D', '0', '-d', '1'])
            elif type == "female":
                subprocess.Popen(['sudo', 'tinyplay', './voice/female.wav', '-D', '0', '-d', '1'])

    def calculate_O_distance(self, point1, point2):
        """ 计算两个坐标点之间的 O 式距离 """
        print(point1)
        print(point2)
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def load_config_file_(self):
        """ 读取 YAML 文件 """
        with open(self.file_path, 'r') as file:
            self.default_arm_params = yaml.safe_load(file)

    def spin_task_(self):
        rclpy.spin(self)

    def lidar_callback_(self, msg):
        """ 单线激光雷达的回调函数 """
        print("激光")
        self.lidar_distance = msg.range

    def timer_work_(self):
        # 更新参数
        # self.get_param_()
        ref = self.get_odom_angle_()
        # 姿态控制
        self.ori_angle_pid.pid_calculate(ref=ref, goal=self.angle)
        self.move_cmd.angular.z = self.ori_angle_pid.out

        # 距离控制
        if self.start_for_pid_distance:
            self.position = self.get_coordinate_value_()

            o_distance = self.get_O_distance()
            o_distance *= self.odom_linear_scale_correction # 修正
            print("在上一次停下后已经行驶的距离: ", o_distance)

            # 计算误差
            self.distance_error = o_distance - abs(self.distance) # 负值控制车向前，正值控制车向后
            print("误差当前值为: ", self.distance_error)

            self.distance_pid.pid_calculate(o_distance, abs(self.distance))
            if self.distance >= 0:
                self.move_cmd.linear.x = self.distance_pid.out
            else:
                self.move_cmd.linear.x = -self.distance_pid.out
            if abs(self.distance_error) < self.distance_tolerance: # 达到目标的情况
                self.start_for_pid_distance = False
                print("任务已完成......")
        elif self.start_for_lidar_distance:
            self.move_cmd.linear.x = self.liear_speed
        else: # 未设定目标的情况
            self.move_cmd.linear.x = 0.0
            self.x_start = self.get_position_().transform.translation.x
            self.y_start = self.get_position_().transform.translation.y
            print("正在停车状态......")
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

        node.set_angle(90.0)
        node.set_distance(1.0)
        node.start_car_and_lidar_controls_stopping(0.05, 0.4)
        node.vision_choose_goal_in_A("a_left")

        # A区
        for i in range(3):
            node.start_car_and_lidar_controls_stopping(0.05, 0.4)
            node.vision_choose_goal_in_A("a_left")
            node.vision_choose_goal_in_A("a_right")

        node.set_distance(-90.0)
        node.start_car_and_lidar_controls_stopping(-0.05, 0.5, 1)
        node.set_angle(0.0)

        # B 区
        for i in range(3):
            node.choose_arm_goal_in_task_alone("moving")
            node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
            node.set_distance(0.1)
            # 前边识别
            node.vision_choose_goal_in_B("b_front")
            node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
            node.set_distance(-0.2)
            # 后边识别
            node.vision_choose_goal_in_B("b_back")



    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()