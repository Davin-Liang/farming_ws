from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Range
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow, radians, degrees
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import PyKDL
from math import pi # 3.14
import yaml
import time
import os
from pid import PID

class Motion_Controller(Node):
    def __init__(self, name):
        super().__init__(name)

        self.ori_angle_pid = PID(0.51, 0.0, 0.126, 1.6, 0.0)
        self.distance_pid = PID(0.42, 0.0, 0.08, 1.0, 0.0)

        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "laser", self.lidar_callback_, 10)
        self.move_cmd = Twist()
        
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

        self.turn_angle     = 0 # 距离上一次停止后车转动的角度
        self.delta_angle    = 0
        self.real_angle = 0.0
        self.last_real_angle = 0.0
        self.total_angle = 0.0
        self.lidar_distance = 0.0

        self.move_direction = "x"
        self.status_of_finishing_goal = True

        time.sleep(5.0)

        # 创建定时器
        self.work_timer = self.create_timer(0.04, self.timer_work_)

    # ---------------- 对外接口函数 ----------------
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
        
    # ----------------------------------------------

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
        node = Motion_Controller("Motion_Controller")
        # rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
