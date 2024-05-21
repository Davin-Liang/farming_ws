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

        self.ori_angle_pid = PID(1.1, 0.012, 0.0, 2.0, 0.1)
        self.distance_pid = PID(0.6, 0.02, 0.0, 0.5, 0.15)

        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.lidar_subcriber_ = self.create_subscription(Range, "laser", self.lidar_callback_, 10)
        self.move_cmd = Twist()

        self.declare_parameter('Kp_ori', 0.51)
        self.ori_angle_pid.Kp = self.get_parameter('Kp_ori').get_parameter_value().double_value
        self.declare_parameter('Ki_ori', 0.0)
        self.ori_angle_pid.Ki = self.get_parameter('Ki_ori').get_parameter_value().double_value
        self.declare_parameter('max_out_ori', 1.6)
        self.ori_angle_pid.max_out = self.get_parameter('max_out_ori').get_parameter_value().double_value
        self.declare_parameter('max_iout_ori', 0.0)
        self.ori_angle_pid.max_iout = self.get_parameter('max_iout_ori').get_parameter_value().double_value
        self.declare_parameter('Kd_ori', 0.126)
        self.ori_angle_pid.Kd = self.get_parameter('Kd_ori').get_parameter_value().double_value

        self.declare_parameter('Kp_distance', 0.42)
        self.distance_pid.Kp = self.get_parameter('Kp_distance').get_parameter_value().double_value
        self.declare_parameter('Ki_distance', 0.0)
        self.distance_pid.Ki = self.get_parameter('Ki_distance').get_parameter_value().double_value
        self.declare_parameter('max_out_distance', 1.0)
        self.distance_pid.max_out = self.get_parameter('max_out_distance').get_parameter_value().double_value
        self.declare_parameter('max_iout_distance', 0.0)
        self.distance_pid.max_iout = self.get_parameter('max_iout_distance').get_parameter_value().double_value
        self.declare_parameter('Kd_distance', 0.08)
        self.distance_pid.Kd = self.get_parameter('Kd_distance').get_parameter_value().double_value

        
        #declare_parameter
        # self.declare_parameter('rate', 20.0)
        # self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.rate = 20.0
        self.set_new_param_to_ros('rate')
        
        # 目标值
        # self.declare_parameter('distance', 1.0)
        # self.distance = self.get_parameter('distance').get_parameter_value().double_value
        # self.declare_parameter('angle', 0.0)
        # self.angle = self.get_parameter('angle').get_parameter_value().double_value
        # self.angle = radians(self.angle)
        self.distance = 0.0
        self.set_new_param_to_ros('distance')
        self.angle = 0.0
        self.set_new_param_to_ros('angle')
        self.angle = radians(self.angle)
        
        # 行驶和转动的速度
        # self.declare_parameter('liear_speed', 0.5)
        # self.liear_speed = self.get_parameter('liear_speed').get_parameter_value().double_value
        self.liear_speed = 0.5
        self.set_new_param_to_ros('liear_speed')
        
        # 阈值
        # self.declare_parameter('distance_tolerance', 0.03)
        # self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.distance_tolerance = 0.03
        self.set_new_param_to_ros('distance_tolerance')
        
        # 修正系数
        # self.declare_parameter('odom_linear_scale_correction', 1.0)
        # self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        # self.declare_parameter('odom_angular_scale_correction', 1.0)
        # self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        self.odom_linear_scale_correction = 1.0
        self.set_new_param_to_ros('odom_linear_scale_correction')
        self.odom_angular_scale_correction = 1.0
        self.set_new_param_to_ros('odom_angular_scale_correction')
        
        self.declare_parameter('start_for_pid_distance', False)
        self.declare_parameter('start_for_lidar_distance', False)
        
        # self.declare_parameter('base_frame', 'base_footprint')
        # self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        # self.declare_parameter('odom_frame','odom')
        # self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = 'base_footprint'
        self.set_new_param_to_ros('base_frame', 'string')
        self.odom_frame = 'odom'
        self.set_new_param_to_ros('odom_frame', 'string')

        self.lidar_threthold = 0.1
        self.set_new_param_to_ros('lidar_threthold')
        
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

        self.move_direction = "x"
        self.status_of_finishing_goal = True

        time.sleep(5.0)

        # 创建定时器
        self.work_timer = self.create_timer(0.04, self.timer_work_)

        self.file_path = os.path.expanduser('~/farming_ws/src/motion_controller/config/position_point.yaml')
        self.load_config_file_()

        self.spin_thread = Thread(target=self.spin_task_)
        self.spin_thread.start()

        print ("Finish init work.")

    # ---------------- 对外接口函数 ----------------
    def set_distance(self, distance):
        """ 设置车轮方向的行驶距离及以什么样的速度行驶 """
        all_new_parameters = []
        self.distance = rclpy.parameter.Parameter('distance', rclpy.Parameter.Type.DOUBLE, distance)
        all_new_parameters.append(self.distance)
        self.start_for_pid_distance = rclpy.parameter.Parameter('start_for_pid_distance', rclpy.Parameter.Type.BOOL, True)
        all_new_parameters.append(self.start_for_pid_distance)
        self.set_parameters(all_new_parameters)

        # 等待完成任务
        while not self.start_for_pid_distance:
            pass


    def set_angle(self, angle):
        """ 设置底盘转动角度 """
        all_new_parameters = []
        self.angle = rclpy.parameter.Parameter('angle', rclpy.Parameter.Type.DOUBLE, angle)
        all_new_parameters.append(self.angle)
        self.set_parameters(all_new_parameters)

        # 等待转完角度
        pass

    def move_based_on_point(self, point_name, direction="x", liear_speed="0.5"):
        """ 根据坐标点来导航 """
        while not self.status_of_finishing_goal: # 等待完成上一个目标
            time.sleep(0.001)
        point = self.points[point_name]
        position = self.get_coordinate_value_()
        print("现在正在前往：", point)
        if direction == "x":
            distance = abs(point[0] - position.x)
            self.set_distance(distance=distance, speed=liear_speed)
        elif direction == "y":
            distance = abs(point[1] - position.y)
            self.set_distance(distance=distance, speed=liear_speed)

    def start_car_and_lidar_controls_stopping(self, speed, threthold=0.1, ignore_num=0):
        """ 开动车并使用单线激光控制小车停止 """
        all_new_parameter = []
        self.liear_speed = rclpy.parameter.Parameter('liear_speed', rclpy.Parameter.Type.DOUBLE, speed)
        all_new_parameter.append(self.liear_speed)
        self.lidar_threthold = rclpy.parameter.Parameter('lidar_threthold', rclpy.Parameter.Type.DOUBLE, threthold)
        all_new_parameter.append(self.lidar_threthold)
        self.start_for_lidar_distance = rclpy.parameter.Parameter('start_for_lidar_distance', rclpy.Parameter.Type.BOOL, True)
        all_new_parameter.append(self.start_for_lidar_distance)
        self.set_parameters(all_new_parameter)
        # all_new_parameter.clear()

        # 等待 car 到位
        time.sleep(1.0) # 保证 car 驶出激光遮挡区域
        real_ignore_num = 0
        print(self.lidar_distance)
        print(self.lidar_threthold)
        while self.lidar_distance > self.lidar_threthold or real_ignore_num != ignore_num:
            if ignore_num == 0:
                pass
            else:
                real_ignore_num += 1
        print("激光已经到达下一个激光遮挡区域")
        all_new_parameters = []
        self.start_for_lidar_distance = rclpy.parameter.Parameter('start_for_lidar_distance', rclpy.Parameter.Type.BOOL, False)
        all_new_parameters.append(self.start_for_lidar_distance)
        self.set_parameters(all_new_parameters)

        # self.wait_thread = Thread(target=self.wait_for_finishing_task_, args=(ignore_num,))
        # self.wait_thread.start()
        
    # ----------------------------------------------

    def wait_for_finishing_task_(self, ignore_num):
        # 等待 car 到位
        time.sleep(1.0) # 保证 car 驶出激光遮挡区域
        for i in range(ignore_num+1):
            while self.lidar_distance > self.lidar_threthold:
                pass
            if self.lidar_distance < self.lidar_threthold:
                time.sleep(1.0)
        all_new_parameters = []
        self.start_for_lidar_distance = rclpy.parameter.Parameter('start_for_lidar_distance', rclpy.Parameter.Type.BOOL, False)
        all_new_parameters.append(self.start_for_lidar_distance)
        self.set_parameters(all_new_parameters)

    def lidar_callback_(self, msg):
        """ 单线激光雷达的回调函数 """
        self.lidar_distance = msg.range

    def timer_work_(self):
        # 更新参数
        self.get_param_()
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
                self.start_for_pid_distance = rclpy.parameter.Parameter('start_for_pid_distance', rclpy.Parameter.Type.BOOL, False)
                all_new_parameters = [self.start_for_pid_distance]
                self.set_parameters(all_new_parameters)
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

    def get_param_(self):
        # self.start_for_pid_distance = self.get_parameter('start_for_pid_distance').get_parameter_value().bool_value
        # self.start_for_lidar_distance = self.get_parameter('start_for_lidar_distance').get_parameter_value().bool_value
        # self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        # self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        # self.distance = self.get_parameter('distance').get_parameter_value().double_value
        # self.angle = self.get_parameter('angle').get_parameter_value().double_value
        # self.angle = radians(self.angle) #角度转成弧度
        # self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        # self.liear_speed = self.get_parameter('liear_speed').get_parameter_value().double_value
        self.update_params_from_ros('start_for_pid_distance', 'bool')
        self.update_params_from_ros('start_for_lidar_distance', 'bool')
        self.update_params_from_ros('odom_angular_scale_correction')
        self.update_params_from_ros('odom_linear_scale_correction')
        self.update_params_from_ros('distance')
        self.update_params_from_ros('angle')
        self.angle = radians(self.angle)
        self.update_params_from_ros('distance_tolerance')
        self.update_params_from_ros('liear_speed')
        self.update_params_from_ros('lidar_threthold')

        self.ori_angle_pid.Kp = self.get_parameter('Kp_ori').get_parameter_value().double_value
        self.ori_angle_pid.Ki = self.get_parameter('Ki_ori').get_parameter_value().double_value
        self.ori_angle_pid.max_out = self.get_parameter('max_out_ori').get_parameter_value().double_value
        self.ori_angle_pid.max_iout = self.get_parameter('max_iout_ori').get_parameter_value().double_value
        self.ori_angle_pid.Kd = self.get_parameter('Kd_ori').get_parameter_value().double_value

        self.distance_pid.Kp = self.get_parameter('Kp_distance').get_parameter_value().double_value
        self.distance_pid.Ki = self.get_parameter('Ki_distance').get_parameter_value().double_value
        self.distance_pid.max_out = self.get_parameter('max_out_distance').get_parameter_value().double_value
        self.distance_pid.max_iout = self.get_parameter('max_iout_distance').get_parameter_value().double_value
        self.distance_pid.Kd = self.get_parameter('Kd_distance').get_parameter_value().double_value
     
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
            # self.real_angle = degrees(angle_rot) + 180.0 # 将 -180~180 转换到 0~360，但程序一启动就是 180
            # if abs(self.real_angle - self.last_real_angle) > 180.0:
            #     if self.real_angle < self.last_real_angle:
            #         self.total_angle += 360.0 - self.last_real_angle + self.real_angle
            #     else:
            #         self.total_angle -= 360.0 - self.real_angle + self.last_real_angle
            # else:
            #     self.total_angle += self.real_angle - self.last_real_angle
            # self.last_real_angle = self.real_angle
            # print("现在角度为", self.total_angle)
            # return radians(self.total_angle)
            return angle_rot
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return       

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def load_config_file_(self):
        """ 读取 YAML 文件 """
        with open(self.file_path, 'r') as file:
            self.points = yaml.safe_load(file)      

    def spin_task_(self):
        rclpy.spin(self) 

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

    def add_variable(self, var_name, value):
        """ 通过一个字符串创建一个类中变量 """
        setattr(self, var_name, value)
    
    def get_variable(self, var_name):
        """ 通过一个字符串读取类中变量的值 """
        return getattr(self, var_name, None)
    
    def update_variable(self, var_name, new_value):
        """ 通过一个字符串更新一个类中变量的值 """
        setattr(self, var_name, new_value)


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
