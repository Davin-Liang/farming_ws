#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import yaml
import time
import subprocess
import copy
from std_msgs.msg import Int32MultiArray, Float64, Bool, Float32MultiArray, Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range
from ai_msgs.msg import PerceptionTargets # type: ignore
from threading import Thread
import asyncio
from math import copysign, sqrt, pow, radians
from pid import PID

class Game_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        # Load arm params
        self.file_path = os.path.expanduser('~/farming_ws/src/motion_controller/config/arm_params.yaml')
        self.load_config_file_()
    
        # Important BOOL value
        self.open_vision_detect         = False # whether to open vision
        self.pre_process                = False # whether to open data primary process
        self.start_for_lidar_distance   = False
        self.start_for_pid_distance     = False
        self.voice_switch               = False
        self.start_count                = False # whether to start counting for error time
        self.error                      = False # whether to pollinate properly
        self.only_arm_action            = False
        self.one_action                 = False
        self.vision_for_voice           = False
        self.data_update                = False
        self.start_delay                = False
        self.start_voice_thread         = False
        self.new_choice                 = False
        self.finish_task                = False
        self.voice_state                = False
        self.normal_lidar_action        = False

        self.arm_params = {'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0} # store real-time arm angles
        self.flowers_with_tag = [] # store flower property
        self.flowers_with_tag_again = []
        self.flowers_lists = [] # primitive flower data
        self.voice_board_params = ['-D', '0', '-d', '0']
        self.closest_flower = []

        # alternative params
        self.area_scaling_factor                     = 0.25
        self.O_distance_threthold_of_judge_same_goal = 99          # the threshold is used to determing whether the identification boxes of the two previous data detections are the same target.
        self.central_point_of_camera                 = [320, 240]
        self.area_of_polliating                      = 80000        # the area threshold for how long the box takes to polinate 70000
        self.joint_speed                             = 2.0          # make the angle of joint rotating as joint speed
        self.threthold_of_x_error                    = 12.0
        self.threthold_of_y_error                    = 12.0
        self.threthold_of_area_error                 = 4500.0
        self.servo_time                              = 880          # Movement time of mechanical arm, unit mm.
        self.servo_reset_time                        = 2000         # Movement time when arm returns to original orientation.
        self.distance_tolerance                      = 0.02
        self.angle_tolerance                         = radians(1.5)
        self.odom_linear_scale_correction            = 1.0
        self.odom_angular_scale_correction           = 1.0
        self.area_difference                         = 50000        #TODO: 修改阈值
        self.time_threshold                          = 5.0          #时间阈值
        self.goal_confidence                         = 0.45
        self.joint2_area_threthold                   = 38000
        self.joint2_coefficient                      = 1.0          #B区
        #self.fix_pose_joint3                         = 120          #TODO:固定姿态3的角度值  首先需调整面积阈值使其达到合适的位置
        # self.fix_pose_joint4                         = 120          #TODO:固定姿态4的角度值
        # self.fix_joint3_angle                        = 5
        # self.fix_joint4_angle                        = 5
        self.add_joint2_pre_slide                    = 6
        self.add_joint3_pre_slide                    = -8
        self.add_joint4_pre_slide                    = -10
        self.add_joint4_slide                        = 9
        self.filter_area                             = 20000
        self.reset_count                             = 0
        self.reset_count_threshold                   = 3

        # publisher and subscriber
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "/hobot_dnn_detection", self.vision_callback_, 10)
        self.joint_angles_publisher_ = self.create_publisher(Float32MultiArray, "/servo_commands", 10)
        self.voice_publisher_ = self.create_publisher(Int32MultiArray, "/voice_commands", 10)
        self.voice_state_subscribe_ = self.create_subscription(Bool, "/voice_state", self.voice_state_callback_, 5)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.buzzer_publisher_ = self.create_publisher(Bool, "/Buzzer", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "/laser", self.lidar_callback_, 10)
        self.odom_subcriber_ = self.create_subscription(Odometry, "/odom_raw", self.odom_callback_, 10)
        self.yaw_angle_subcriber_ = self.create_subscription(Float64, "/yaw_angle", self.yaw_angle_callback_, 10)
        self.car_command_publisher_ = self.create_publisher(Float64MultiArray, "/car_commands", 10)
        self.finish_task_subcriber_ = self.create_subscription(Bool, "/finish_task", self.finish_task_callback_, 10)

        self.move_cmd           = Twist()
        self.angles_of_joints   = Float32MultiArray()
        self.voice_cmd          = Int32MultiArray()
        self.buzzer_cmd         = Bool()
        self.car_command_cmd    = Float64MultiArray()

        self.ori_angle_pid = PID(Kp=0.698, Ki=0.0003, Kd=0.456, max_out=0.125, max_iout=0.0065) #0.15
        self.distance_pid  = PID(Kp=0.45, Ki=0.0, Kd=0.1, max_out=0.04, max_iout=0.00025)

        self.female_num      = 0
        self.distance        = 0.0
        self.angle           = 0.0
        self.yaw_angle       = 0.0
        self.lidar_threthold = 0.1
        self.liear_speed     = 0.5
        self.start_count_time = 0.0
        self.joint_last_state  = {}
        self.pose_name       = ""
        self.angle = radians(self.angle)
        self.deviation_angle = radians(0.85)

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y

        self.distance_error = 0
        self.angle_error    = 0

        time.sleep(1.0)

        self.work_timer = self.create_timer(0.002, self.timer_work_)
        self.arm_timer = self.create_timer(0.5, self.arm_timer_callback_)
        # self.voice_timer = self.create_timer(0.01, self.voice_task_)

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

        # self.voice_thread = asyncio.create_task(self.voice_task_)
        # self.voice_thread = Thread(target=self.voice_task_)
        # self.voice_thread.start()

        # self.arm_thread = Thread(target=self.arm_timer_callback_)
        # self.arm_thread.start()
    
        self.guo_xiaoyu_is_broadcasting("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Finish initial task !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.guo_xiaoyu_is_broadcasting("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Finish initial task !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.guo_xiaoyu_is_broadcasting("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Finish initial task !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        self.guo_xiaoyu_is_broadcasting("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Finish initial task !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    def voice_task_(self):
        # while True:
        if self.start_voice_thread:
            if self.voice_switch:
                self.voice_(self.flowers_lists)
            self.start_voice_thread = False

    def finish_task_callback_(self, msg):
        if msg.data == True:
            self.finish_task = True

# -----------------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------External interface funxtion--------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------
    def auto_pollinate(self, place="A", arm_pose="a_left", joint2_area_threthold=38000, joint2_coefficient=1.5,area_of_polliating=70000):
        self.area_of_polliating = area_of_polliating
        self.joint2_area_threthold = joint2_area_threthold
        self.joint2_coefficient = joint2_coefficient
        if "A" == place or "C" == place:
            self.vision_control_arm(place, arm_pose)
            if self.error:
                if self.new_choice:
                    self.guo_xiaoyu_is_broadcasting("重新给次机会!!!!!!")
                    self.find_next_arm_goal_on_position()
                else:
                    self.error = False
                    for index, flower_with_tag in enumerate(self.flowers_with_tag):
                        if flower_with_tag['Moving'] == True:
                            self.flowers_with_tag_again[index]['Moving'] = False
                            self.flowers_with_tag_again[index]['Pollinated'] = True
                
            if self.only_arm_action:
                return
            if self.one_action:
                return
            if self.vision_for_voice:
                return
            for i in range(self.female_num-1):
                print(self.female_num)
                self.find_next_arm_goal_on_position()
                # print("self.error = ", self.error)
                if self.error:
                    if self.new_choice:
                        self.guo_xiaoyu_is_broadcasting("重新给次机会!!!!!!")
                        self.find_next_arm_goal_on_position()
                    else:
                        self.error = False
                        for index, flower_with_tag in enumerate(self.flowers_with_tag):
                            if flower_with_tag['Moving'] == True:
                                self.flowers_with_tag_again[index]['Moving'] = False
                                self.flowers_with_tag_again[index]['Pollinated'] = True
        elif "B" == place:
            self.vision_control_arm(place, arm_pose)
            if self.error:
                if self.new_choice:
                    self.guo_xiaoyu_is_broadcasting("重新给次机会!!!!!!")
                    self.find_next_arm_goal_on_position()
                else:
                    self.error = False
                    for index, flower_with_tag in enumerate(self.flowers_with_tag):
                        if flower_with_tag['Moving'] == True:
                            self.flowers_with_tag_again[index]['Moving'] = False
                            self.flowers_with_tag_again[index]['Pollinated'] = True
    
    def guo_xiaoyu_is_broadcasting(self, info):
        self.get_logger().info(info)

    def buzzer_tips(self, times=1.0):
        """ Make buzzer ring for a second times. """
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

        self.guo_xiaoyu_is_broadcasting('Waiting for finishing task......')
        while self.open_vision_detect:
            pass
        if self.place_name == 'C':
            while not self.voice_state:
                # print("voice_announce")
                pass
            self.voice_state = False
        # self.guo_xiaoyu_is_broadcasting('Finished task!!!!!!')

    def reset_vision_data(self):
        self.flowers_with_tag.clear()
        self.flowers_with_tag_again.clear()
        self.voice_state = False
        self.female_num = 0
        self.reset_count = 0

    def find_next_arm_goal_on_position(self):
        self.joint_last_state = {}
        self.open_vision_detect = True
        self.pre_process = True

        self.guo_xiaoyu_is_broadcasting('Waiting for finishing task......')
        while self.open_vision_detect:
            pass
        # self.guo_xiaoyu_is_broadcasting('Finished task!!!!!!')

    def set_distance(self, distance):
        self.distance = distance
        self.start_for_pid_distance = True

        self.car_command_cmd.data = []
        self.car_command_cmd.data.append(0.0)
        self.car_command_cmd.data.append(distance)
        self.car_command_cmd.data.append(0.0)
        self.car_command_publisher_.publish(self.car_command_cmd)
        self.car_command_publisher_.publish(self.car_command_cmd)
        self.car_command_publisher_.publish(self.car_command_cmd)

        self.guo_xiaoyu_is_broadcasting('Waiting for finishing task......')

        # while self.finish_task == False:
        #     pass
        # self.finish_task = False

        while self.start_for_pid_distance:
            pass
        # self.guo_xiaoyu_is_broadcasting('Finished task!!!!!!')
        # time.sleep(2.0)

    def set_angle(self, angle, times=4.0):
        # time.sleep(times-1.0)
        self.angle = radians(angle)

        self.car_command_cmd.data = []
        self.car_command_cmd.data.append(2.0)
        self.car_command_cmd.data.append(radians(angle))
        self.car_command_cmd.data.append(0.0)
        self.car_command_publisher_.publish(self.car_command_cmd)
        self.car_command_publisher_.publish(self.car_command_cmd)
        self.car_command_publisher_.publish(self.car_command_cmd)

        self.guo_xiaoyu_is_broadcasting('Waiting for finishing task......')
        while abs(self.angle-self.yaw_angle) > self.angle_tolerance:
            pass
        self.guo_xiaoyu_is_broadcasting('Finished task!!!!!!')
        # time.sleep(4.0)
        time.sleep(times)

    def car_action_in_lidar(self, speed, threthold=0.1, distance_threshold=0.25):
        """ Start car and stop car by lidar. """
        while abs(self.angle-self.yaw_angle) > self.angle_tolerance:
            self.guo_xiaoyu_is_broadcasting("****************************")

        self.liear_speed = speed
        self.lidar_threthold = threthold
        self.start_delay = True
        self.start_for_lidar_distance = True

        self.car_command_cmd.data = []
        self.car_command_cmd.data.append(1.0)
        self.car_command_cmd.data.append(speed)
        self.car_command_cmd.data.append(threthold)
        self.car_command_publisher_.publish(self.car_command_cmd)
        self.car_command_publisher_.publish(self.car_command_cmd)
        self.car_command_publisher_.publish(self.car_command_cmd)

        self.guo_xiaoyu_is_broadcasting('Waiting for finishing task......')
        # time.sleep(3.0) # ensure car will leave the area of lidar keeping out.

        # while self.finish_task == False:
        #     pass
        # print("finish_task")
        # self.finish_task = False

        while self.get_O_distance_() < distance_threshold:
            print("Current distance = ", self.get_O_distance_())
            pass
        self.guo_xiaoyu_is_broadcasting("avoided error lidar data!!!!!!")
        self.normal_lidar_action = True

        while self.start_for_lidar_distance:
            pass
        self.guo_xiaoyu_is_broadcasting('Finished task!!!!!!')
        time.sleep(2.0)

    def choose_arm_goal(self, pose_name):
        """ Use arm goals in YAML file. """
        self.arm_params['joint1'] = float(self.default_arm_params['joint1_'+pose_name])
        self.arm_params['joint2'] = float(self.default_arm_params['joint2_'+pose_name])
        self.arm_params['joint3'] = float(self.default_arm_params['joint3_'+pose_name])
        self.arm_params['joint4'] = float(self.default_arm_params['joint4_'+pose_name])
        self.angles_of_joints.data = []
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.angles_of_joints.data.append(self.servo_reset_time)
        self.joint_angles_publisher_.publish(self.angles_of_joints)

        if self.pose_name != pose_name:
            self.pose_name = pose_name
            time.sleep(3.0)

    def choose_arm_goal_in_number(self, joint1=0, joint2=0, joint3=0, joint4=0, servo_time=250):
        """ Use own numbers to control arm instead of arm goals in YAML file. """
        if joint1 != 0:
            self.arm_params['joint1'] = joint1
        if joint2 != 0:
            self.arm_params['joint2'] = joint2
        if joint3 != 0:
            self.arm_params['joint3'] = joint3
        if joint4 != 0:
            self.arm_params['joint4'] = joint4
        self.angles_of_joints.data = []
        self.angles_of_joints.data.append(self.arm_params['joint1'])
        self.angles_of_joints.data.append(self.arm_params['joint2'])
        self.angles_of_joints.data.append(self.arm_params['joint3'])
        self.angles_of_joints.data.append(self.arm_params['joint4'])
        self.angles_of_joints.data.append(servo_time)
        self.joint_angles_publisher_.publish(self.angles_of_joints)
        print(servo_time/1000 + 0.25)
        time.sleep(servo_time/1000 + 0.25)

# -----------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------END------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------------

    def confrim_moving_goal_for_arm_(self, flowers_lists):
        # 数据预处理并选择第一个处理的目标
        self.data_pre_processing_(flowers_lists)

        if self.vision_for_voice == True:
            self.open_vision_detect = False
            return

        have_female = False

        if self.place_name == 'B':
            for index, flower in enumerate(flowers_lists):
                if flower['Area'] < self.filter_area:
                    flowers_lists.pop(index)


        # Update data
        for flower in flowers_lists:
            if flower['Type'] == "famale":
                have_female = True
                print("===============================================================================")
                self.guo_xiaoyu_is_broadcasting("该次处理的目标为雌花!!!!!!")
                for index, flower_with_tag in enumerate(self.flowers_with_tag):
                    if flower_with_tag['Moving'] == True:
                        if self.calculate_O_distance_(flower_with_tag['CentralPoint'], flower['CentralPoint']) < self.O_distance_threthold_of_judge_same_goal:
                            self.guo_xiaoyu_is_broadcasting("满足 O 式距离限制!!!!!!")
                            if abs(flower_with_tag['Area']-flower['Area']) <= self.area_difference:
                                self.guo_xiaoyu_is_broadcasting("满足面积变化限制!!!!!!")
                                self.flowers_with_tag[index]['CentralPoint'] = flower['CentralPoint']
                                self.flowers_with_tag[index]['Area'] = flower['Area']
                                self.control_arm_()
                                self.reset_count = 0
                                break # hop inner loop

                            else:
                                self.guo_xiaoyu_is_broadcasting("Area way!!!")
                                self.closest_flower.clear()
                                self.closest_flower = self.find_closest_flower(flowers_lists, flower_with_tag)
                                self.flowers_with_tag[index]['CentralPoint'] = self.closest_flower['CentralPoint']
                                self.flowers_with_tag[index]['Area'] = self.closest_flower['Area']
                                self.reset_count += 1
                                break
                        else:
                            self.guo_xiaoyu_is_broadcasting("O_distance way!!!")
                            self.closest_flower.clear()
                            self.closest_flower = self.find_closest_flower(flowers_lists, flower_with_tag)
                            self.flowers_with_tag[index]['CentralPoint'] = self.closest_flower['CentralPoint']
                            self.flowers_with_tag[index]['Area'] = self.closest_flower['Area']
                            self.reset_count += 1
                            break

        if have_female == False:
            self.guo_xiaoyu_is_broadcasting("Only have male!!!!!!")
            self.reset_count += 1

    def data_pre_processing_(self, flowers_lists):
        """ 为存储花属性的字典添加花属性：是否正在操作、是否已授粉 """
        # 类型、中心的坐标、面积、是否正在操作、是否已授粉
        flower_with_tag = {'Type': '', 'CentralPoint': [], 'Area': 0, 'Moving': False, 'Pollinated': False}
        if self.pre_process:
            if len(self.flowers_with_tag) == 0:
                print("正在进行数据预处理")
                self.voice_process(self.flowers_lists)
                for index, flower in enumerate(flowers_lists):
                    if self.place_name == 'B':
                        if  flower['Area'] > self.filter_area:
                            if flower['Type'] == 'famale':
                                self.female_num += 1
                                flower_with_tag['Type'] = flower['Type']
                                flower_with_tag['CentralPoint'] = flower['CentralPoint']
                                flower_with_tag['Area'] = flower['Area']
                                if self.arm_moving == False:
                                    flower_with_tag['Moving'] = True
                                    self.arm_moving = True
                                else:
                                    flower_with_tag['Moving'] = False
                                self.flowers_with_tag.append(copy.deepcopy(flower_with_tag))
                                break
                    else:
                        if flower['Type'] == 'famale':
                            self.female_num += 1
                            if self.female_num >= 3:
                                self.female_num = 3
                            flower_with_tag['Type'] = flower['Type']
                            flower_with_tag['CentralPoint'] = flower['CentralPoint']
                            flower_with_tag['Area'] = flower['Area']
                            if self.arm_moving == False:
                                flower_with_tag['Moving'] = True
                                self.arm_moving = True
                            else:
                                flower_with_tag['Moving'] = False
                            self.flowers_with_tag.append(copy.deepcopy(flower_with_tag))
                # if self.female_num == 0:
                #     self.reset_arm_pose_()
                # self.flowers_with_tag_again = copy.deepcopy(self.flowers_with_tag)
                #添加语音播报
                # self.start_voice_thread = True
                # print("voice_pre")
                # self.voice_process(self.flowers_lists)
                # if self.voice_switch:
                #     self.voice_(flowers_lists)
                if self.female_num == 0:
                    self.voice_state = True
                    self.reset_arm_pose_("no")
                self.flowers_with_tag_again = copy.deepcopy(self.flowers_with_tag)
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
                if not self.error:
                # 寻找未“授粉”的花，找到的第一朵就设置为目标
                    for index, flower_with_tag in enumerate(self.flowers_with_tag):
                        if flower_with_tag['Pollinated'] != True:
                            self.flowers_with_tag[index]['Moving'] = True
                            break
            self.error = False
        self.pre_process = False # 关闭数据预处理

    def control_arm_(self):
        y_error = 0
        x_error = 0
        area_error = 0
        area = 0
        # print(self.flowers_with_tag)
        for flower_with_tag in self.flowers_with_tag:
            if flower_with_tag['Moving'] == True:
                x_error = self.central_point_of_camera[0] - flower_with_tag['CentralPoint'][0]
                y_error = self.central_point_of_camera[1] - flower_with_tag['CentralPoint'][1]
                area_error = self.area_of_polliating - flower_with_tag['Area']
                area = flower_with_tag['Area']
                break
        self.angles_of_joints.data = []
        # print(x_error)
        # print(area_error)

        # if abs(x_error) > self.threthold_of_x_error:
        #     self.arm_params['joint1'] = float(self.limit_num_(self.arm_params['joint1'] + copysign(0.75 * self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
        # self.angles_of_joints.data.append(self.arm_params['joint1'])

        if self.place_name == 'A':
            if area < self.joint2_area_threthold:                                    #38000
                if abs(x_error) > self.threthold_of_x_error:
                    self.arm_params['joint1'] = float(self.limit_num_(self.arm_params['joint1'] + copysign(0.3 * self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint1'])

                if abs(area_error) > self.threthold_of_area_error:
                    self.arm_params['joint2'] = float(self.limit_num_(self.arm_params['joint2'] + copysign(self.joint2_coefficient * self.joint_speed, area_error), self.default_arm_params['joint2_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint2']) # 1.5

                if abs(y_error) > self.threthold_of_y_error:
                    self.arm_params['joint3'] = float(self.limit_num_(self.arm_params['joint3'] + copysign(0.25 * self.joint_speed, y_error), self.default_arm_params['joint3_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint3'])

                if abs(y_error) > self.threthold_of_y_error:
                    self.arm_params['joint4'] = float(self.limit_num_(self.arm_params['joint4'] + copysign(1.0 * self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint4'])
            else:
                if abs(x_error) > self.threthold_of_x_error:
                    self.arm_params['joint1'] = float(self.limit_num_(self.arm_params['joint1'] + copysign(0.15 * self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
                else:
                    self.guo_xiaoyu_is_broadcasting("x error is suitable!!!")
                self.angles_of_joints.data.append(self.arm_params['joint1'])

                self.angles_of_joints.data.append(self.arm_params['joint2'])
                if abs(area_error) > self.threthold_of_area_error:
                    self.arm_params['joint3'] = float(self.limit_num_(self.arm_params['joint3'] + copysign(0.75 * self.joint_speed, -area_error), self.default_arm_params['joint3_limiting']))
                else:
                    self.guo_xiaoyu_is_broadcasting("area error is suitable!!!")
                self.angles_of_joints.data.append(self.arm_params['joint3'])

                if abs(y_error) > self.threthold_of_y_error:
                    self.arm_params['joint4'] = float(self.limit_num_(self.arm_params['joint4'] + copysign(0.4 * self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
                else:
                    self.guo_xiaoyu_is_broadcasting("y error is suitable!!!")
                self.angles_of_joints.data.append(self.arm_params['joint4'])
        elif self.place_name == 'C':
            print("c!!!!!!!!!!!!!!!!!!!")
            if area < self.joint2_area_threthold:                                    #38000
                if abs(x_error) > self.threthold_of_x_error:
                    self.arm_params['joint1'] = float(self.limit_num_(self.arm_params['joint1'] + copysign(0.6 * self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint1'])

                if abs(area_error) > self.threthold_of_area_error:
                    self.arm_params['joint2'] = float(self.limit_num_(self.arm_params['joint2'] + copysign(self.joint2_coefficient * self.joint_speed, area_error), self.default_arm_params['joint2_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint2']) # 1.5

                # if abs(y_error) > self.threthold_of_y_error:
                #     self.arm_params['joint3'] = float(self.limit_num_(self.arm_params['joint3'] + copysign(0.25 * self.joint_speed, y_error), self.default_arm_params['joint3_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint3'])

                if abs(y_error) > self.threthold_of_y_error:
                    self.arm_params['joint4'] = float(self.limit_num_(self.arm_params['joint4'] + copysign(2.0 * self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint4'])
            else:
                if abs(x_error) > self.threthold_of_x_error:
                    self.arm_params['joint1'] = float(self.limit_num_(self.arm_params['joint1'] + copysign(0.3 * self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
                else:
                    self.guo_xiaoyu_is_broadcasting("x error is suitable!!!")
                self.angles_of_joints.data.append(self.arm_params['joint1'])

                self.angles_of_joints.data.append(self.arm_params['joint2'])

                if abs(area_error) > self.threthold_of_area_error:
                    self.arm_params['joint3'] = float(self.limit_num_(self.arm_params['joint3'] + copysign(0.6 * self.joint_speed, -area_error), self.default_arm_params['joint3_limiting']))
                else:
                    self.guo_xiaoyu_is_broadcasting("area error is suitable!!!")
                self.angles_of_joints.data.append(self.arm_params['joint3'])

                if abs(y_error) > self.threthold_of_y_error:#1.25
                    self.arm_params['joint4'] = float(self.limit_num_(self.arm_params['joint4'] + copysign(0.75 * self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
                else:
                    self.guo_xiaoyu_is_broadcasting("y error is suitable!!!")
                self.angles_of_joints.data.append(self.arm_params['joint4'])
        elif self.place_name == 'B':
            print("B !!!!!!!!!!!!!!!!!!!!!")
            if area < self.joint2_area_threthold:                                    #38000
                if abs(x_error) > self.threthold_of_x_error:
                    self.arm_params['joint1'] = float(self.limit_num_(self.arm_params['joint1'] + copysign(0.2 * self.joint_speed, x_error), self.default_arm_params['joint1_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint1'])

                if abs(area_error) > self.threthold_of_area_error:
                    self.arm_params['joint2'] = float(self.limit_num_(self.arm_params['joint2'] + copysign(self.joint2_coefficient * self.joint_speed, area_error), self.default_arm_params['joint2_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint2']) # 1.5

                # if abs(y_error) > self.threthold_of_y_error:
                #     self.arm_params['joint3'] = float(self.limit_num_(self.arm_params['joint3'] + copysign(0.25 * self.joint_speed, y_error), self.default_arm_params['joint3_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint3'])

                if abs(y_error) > self.threthold_of_y_error:
                    self.arm_params['joint4'] = float(self.limit_num_(self.arm_params['joint4'] + copysign(0.3 * self.joint_speed, y_error), self.default_arm_params['joint4_limiting']))
                self.angles_of_joints.data.append(self.arm_params['joint4'])
            



        if (abs(x_error) < self.threthold_of_x_error and
            abs(area_error) < self.threthold_of_area_error and
            abs(y_error) < self.threthold_of_y_error):
            self.guo_xiaoyu_is_broadcasting('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Finished pollinating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.guo_xiaoyu_is_broadcasting('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Finished pollinating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.guo_xiaoyu_is_broadcasting('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Finished pollinating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

            for index, flower_with_tag in enumerate(self.flowers_with_tag):
                if flower_with_tag['Moving'] == True:
                    self.flowers_with_tag_again[index]['Moving'] = False
                    self.flowers_with_tag_again[index]['Pollinated'] = True
            self.slide_draw()
            self.reset_arm_pose_(self.pose_name)
            return
        print(self.angles_of_joints)
        self.angles_of_joints.data.append(self.servo_time)
        self.joint_angles_publisher_.publish(self.angles_of_joints)

    def limit_num_(self, num, num_range):
        if num > num_range[1]:
            num = num_range[1]
        elif num < num_range[0]:
            num = num_range[0]
        return num

    def slide_draw(self):
        self.choose_arm_goal_in_number(self.arm_params['joint1'], 
                                       self.arm_params['joint2'], 
                                       self.arm_params['joint3'], 
                                       self.arm_params['joint4']+self.add_joint4_pre_slide)
        self.choose_arm_goal_in_number(self.arm_params['joint1'], 
                                       self.arm_params['joint2'], 
                                       self.arm_params['joint3']+self.add_joint3_pre_slide, 
                                       self.arm_params['joint4'])
        self.choose_arm_goal_in_number(self.arm_params['joint1'], 
                                       self.arm_params['joint2']+self.add_joint2_pre_slide, 
                                       self.arm_params['joint3'], 
                                       self.arm_params['joint4'],
                                       1000)
        print('滑动')
        self.choose_arm_goal_in_number(self.arm_params['joint1'],
                                       self.arm_params['joint2'],
                                       self.arm_params['joint3'],
                                       self.arm_params['joint4']+self.add_joint4_slide)

        if self.place_name == "B":
            # self.arm_params['joint1'] = float(self.default_arm_params['joint1_'+self.pose_name])
            self.arm_params['joint2'] = float(self.default_arm_params['joint2_'+self.pose_name])
            # self.arm_params['joint3'] = float(self.default_arm_params['joint3_'+self.pose_name])
            # self.arm_params['joint4'] = float(self.default_arm_params['joint4_'+self.pose_name])
            self.angles_of_joints.data = []
            self.angles_of_joints.data.append(self.arm_params['joint1'])
            self.angles_of_joints.data.append(self.arm_params['joint2'])
            self.angles_of_joints.data.append(self.arm_params['joint3'])
            self.angles_of_joints.data.append(self.arm_params['joint4'])
            self.angles_of_joints.data.append(500)
            self.joint_angles_publisher_.publish(self.angles_of_joints)

            time.sleep(1.0)

    def reset_arm_pose_(self, pose="a_left"):
        """ 控制 arm 回到初始姿态 """
        self.guo_xiaoyu_is_broadcasting('Control arm to return initial pose......')
        if pose != "no":
            self.choose_arm_goal(pose)
        self.open_vision_detect = False
        self.pre_process = False
        self.data_update = False
        if pose != "no":
            time.sleep(3.5)

    def voice_process(self, flowers_lists):
        # goal_list_agn = []
        self.voice_cmd.data = []
        #数组第一位为播报哪个区域
        if self.place_name == 'A':
            # goal_list_agn.append(0)     # A区
            self.voice_cmd.data.append(0)
            goal_list = self.data_sort_(flowers_lists, prior_axis='v')
        if self.place_name == 'B':
            # goal_list_agn.append(1)     # B区
            self.voice_cmd.data.append(1)
            goal_list = self.data_sort_(flowers_lists, prior_axis='v')
        if self.place_name == 'C':
            # goal_list_agn.append(2)     # C区
            self.voice_cmd.data.append(2)
            goal_list = self.data_sort_(flowers_lists, prior_axis='h')
        for index, goal in enumerate(goal_list):
            if index == 0:
                    if goal == 'famale':
                        # goal_list_agn.append(1)
                        self.voice_cmd.data.append(1)
                    else:
                        # goal_list_agn.append(2)
                        self.voice_cmd.data.append(2)
            if index == 1:
                    if goal == 'famale':
                        # goal_list_agn.append(1)
                        self.voice_cmd.data.append(1)
                    else:
                        # goal_list_agn.append(2)
                        self.voice_cmd.data.append(2)
            if index == 2:
                    if goal == 'famale':
                        # goal_list_agn.append(1)
                        self.voice_cmd.data.append(1)
                    else:
                        # goal_list_agn.append(2)
                        self.voice_cmd.data.append(2)
        # self.voice_cmd.data.append(goal_list_agn)
        self.voice_publisher_.publish(self.voice_cmd)
        # print("voice_publish")

    def voice_(self, flowers_lists):
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

    def find_closest_flower(self, flowers_lists, flowers_with_tag):
        """找到与指定花朵的CentralPoint差值最小的花朵"""
        min_distance = float('inf')
        # print(flowers_lists)
        closest_flower = None
        for tagged_flower in flowers_lists:
            # print(tagged_flower)
            dist = self.calculate_O_distance_(flowers_with_tag['CentralPoint'], tagged_flower['CentralPoint'])
            if dist < min_distance:
                min_distance = dist
                # closest_flower = tagged_flower 
                closest_flower = copy.deepcopy(tagged_flower)
        return closest_flower

    def voice_broadcast(self, direction='', type=''):
        """ 语音播报 """
        if direction != '':
            if direction == 'up':
                self.guo_xiaoyu_is_broadcasting('The flower above is: ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/up.wav'] + self.voice_board_params)
                # time.sleep(2.0)
                # await asyncio.sleep(2.0)
            elif direction == 'middle':
                self.guo_xiaoyu_is_broadcasting('The middle flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/middle.wav'] + self.voice_board_params)
                # time.sleep(2.0)
            elif direction == 'down':
                self.guo_xiaoyu_is_broadcasting('The lower flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/down.wav'] + self.voice_board_params)
                # time.sleep(2.0)
            elif direction == 'left':
                self.guo_xiaoyu_is_broadcasting('The left flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/left.wav'] + self.voice_board_params)
                # time.sleep(2.0)
            elif direction == 'right':
                self.guo_xiaoyu_is_broadcasting('The right flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/right.wav'] + self.voice_board_params)
                # time.sleep(2.0)
        if type != '':
            if type == 'male':
                self.guo_xiaoyu_is_broadcasting('male!!! male!!! male!!! ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/male.wav'] + self.voice_board_params)
                # time.sleep(1.0)
            elif type == 'female':
                self.guo_xiaoyu_is_broadcasting('female!!! female!!! female!!! ')
                subprocess.Popen(['sudo', 'tinyplay', './voice/female.wav'] + self.voice_board_params)
                # time.sleep(1.0)

    def calculate_O_distance_(self, point1, point2):
        return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def load_config_file_(self):
        """ Load YAML file. """
        with open(self.file_path, 'r') as file:
            self.default_arm_params = yaml.safe_load(file)

    def get_O_distance_(self):
        """ Calculate the O distance from current positon to the positon of starting action. """
        return sqrt(pow((self.position.x - self.x_start), 2) + pow((self.position.y - self.y_start), 2))


    def yaw_angle_callback_(self, msg):
        self.yaw_angle = -msg.data

    def odom_callback_(self, msg):
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y

    def arm_timer_callback_(self):
        # while True:
        # print("start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        x = time.time()

        if not self.open_vision_detect:
            return
        
        if not self.data_update:
            return

        if self.vision_for_voice:
            self.voice_switch = True

        # print("self.joint_last_state = ", self.joint_last_state)
        # print("self.arm_params = ", self.arm_params)

        if self.reset_count > self.reset_count_threshold:
            # if self.joint_last_state == self.arm_params:
            self.error = True
            self.voice_state = True
            self.guo_xiaoyu_is_broadcasting("目标点丢失!!!!!!")
            # if self.vision_for_voice != True:
                # if self.start_count == False:
                #     self.start_count = True
                #     self.start_count_time = time.time()
                # if self.start_count == True:
                #     if time.time() - self.start_count_time > self.time_threshold:
                        # self.start_count = False
                # self.slide_draw()
            self.reset_arm_pose_(self.pose_name)
        else:
        # print(self.flowers_lists)
            # self.joint_last_state = copy.deepcopy(self.arm_params)
            # print("'self.flowers_lists' 's length = ", self.flowers_lists)
            if 0 != len(self.flowers_lists):
                # print(self.flowers_lists)
                # self.start_count = False
                self.confrim_moving_goal_for_arm_(self.flowers_lists)
            else:
                self.guo_xiaoyu_is_broadcasting("Why not have data?!?!?!")
                self.reset_count += 1 

        y = time.time()
        # print("time = ", y - x)
        # time.sleep(0.01)
                

    def vision_callback_(self, msg):
        """ Get type, central point, area of each goal frame from callback function. """
        # print("fadflakdjflajdfladsjflasdjfl")
        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0}
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                flower['CentralPoint'].clear()
                if msg.targets[i].type == "male" or msg.targets[i].type == "famale": 
                    if msg.targets[i].rois[0].confidence > self.goal_confidence:
                        flower['Type'] = msg.targets[i].type
                        flower['CentralPoint'].append(msg.targets[i].rois[0].rect.x_offset + msg.targets[i].rois[0].rect.width/2)
                        flower['CentralPoint'].append(msg.targets[i].rois[0].rect.y_offset + msg.targets[i].rois[0].rect.height/2)
                        flower['Area'] = msg.targets[i].rois[0].rect.height * msg.targets[i].rois[0].rect.width
                        # Get original data.
                        flowers_lists.append(copy.deepcopy(flower))
                        # print(flower)
        self.flowers_lists.clear()
        self.flowers_lists = copy.deepcopy(flowers_lists)
        # print(self.flowers_lists)

        self.data_update = True
        # print(self.flowers_lists)

    def spin_task_(self):
        """ Spin node in class. """
        rclpy.spin(self)

    def lidar_callback_(self, msg):
        """ Get distance of lidar from callback function. """
        self.lidar_distance = msg.range

    def voice_state_callback_(self, msg):
        self.voice_state = msg.data

    def timer_work_(self):
        # orientation control
        self.ori_angle_pid.pid_calculate(ref=self.yaw_angle, goal=self.angle)
        self.move_cmd.angular.z = self.ori_angle_pid.out

        # distance control
        x = time.time()
        if self.start_for_pid_distance:
            o_distance = self.odom_linear_scale_correction * self.get_O_distance_()
            # calculate error
            self.distance_pid.pid_calculate(ref=o_distance, goal=abs(self.distance))
            self.move_cmd.linear.x = copysign(self.distance_pid.out, self.distance)
            if self.distance < 0:
                self.move_cmd.linear.x = -self.distance_pid.out
            else:
                self.move_cmd.linear.x = self.distance_pid.out
            print("self.distance_pid.out = ", self.distance_pid.out)
            print("self.move_cmd.linear.x = ", self.move_cmd.linear.x)
            print("Error = ", o_distance - abs(self.distance))
            
            if abs(o_distance - abs(self.distance)) < self.distance_tolerance: # achieve goal
                self.guo_xiaoyu_is_broadcasting('Arrived goal distance !!!!!!')
                self.guo_xiaoyu_is_broadcasting('Arrived goal distance !!!!!!')
                self.guo_xiaoyu_is_broadcasting('Arrived goal distance !!!!!!')
                self.cmd_vel.publish(Twist())
                self.start_for_pid_distance = False
                # if self.start_count == False:
                #     self.start_count_time = time.time()
                #     self.start_count = True
                # if self.start_count:
                #     if time.time() - self.start_count_time > self.time_threshold:
                #         self.start_count = False
                #         self.guo_xiaoyu_is_broadcasting('Arrived goal distance !!!!!!')
                #         self.guo_xiaoyu_is_broadcasting('Arrived goal distance !!!!!!')
                #         self.guo_xiaoyu_is_broadcasting('Arrived goal distance !!!!!!')
                #         self.start_for_pid_distance = False
                #         self.x_start = self.position.x
                #         self.y_start = self.position.y
            else:
                self.guo_xiaoyu_is_broadcasting("Accidently overstepped the limit.")
                self.start_count = False
        elif self.start_for_lidar_distance:
            # if self.start_delay:
            #     self.move_cmd.linear.x = self.liear_speed
            #     self.cmd_vel.publish(self.move_cmd)
            #     time.sleep(3.0)
            #     self.start_delay = False
            self.move_cmd.linear.x = self.liear_speed
            if self.lidar_distance < self.lidar_threthold and self.normal_lidar_action:
                self.guo_xiaoyu_is_broadcasting('Finished task!!!!!!')
                self.start_for_lidar_distance = False
                self.normal_lidar_action = False
                self.cmd_vel.publish(Twist())
        else:
            self.move_cmd.linear.x = 0.0
            self.x_start = self.position.x
            self.y_start = self.position.y
        self.cmd_vel.publish(self.move_cmd)
        # y = time.time()
        # print("time of timer work = ", y - x)
        






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

