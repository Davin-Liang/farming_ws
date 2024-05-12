import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow, radians
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

        self.declare_parameter('Kp_distance', 1.5)
        self.distance_pid.Kp = self.get_parameter('Kp_distance').get_parameter_value().double_value
        self.declare_parameter('Ki_distance', 0.005)
        self.distance_pid.Ki = self.get_parameter('Ki_distance').get_parameter_value().double_value
        self.declare_parameter('max_out_distance', 2.0)
        self.distance_pid.max_out = self.get_parameter('max_out_distance').get_parameter_value().double_value
        self.declare_parameter('max_iout_distance', 0.02)
        self.distance_pid.max_iout = self.get_parameter('max_iout_distance').get_parameter_value().double_value
        self.declare_parameter('Kd_distance', 0.0)
        self.distance_pid.Kd = self.get_parameter('Kd_distance').get_parameter_value().double_value

        
        #declare_parameter
        self.declare_parameter('rate', 20.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        
        # 目标值
        self.declare_parameter('distance', 1.0)
        self.distance = self.get_parameter('distance').get_parameter_value().double_value
        self.declare_parameter('angle', 0.0)
        self.angle = self.get_parameter('angle').get_parameter_value().double_value
        self.angle = radians(self.angle)
        
        # 行驶和转动的速度
        self.declare_parameter('liear_speed', 0.5)
        self.liear_speed = self.get_parameter('liear_speed').get_parameter_value().double_value
        self.declare_parameter('angular_speed', 0.5)
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # 阈值
        self.declare_parameter('distance_tolerance', 0.03)
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.declare_parameter('angle_tolerance',1.5)
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        
        # 修正系数
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        self.declare_parameter('odom_angular_scale_correction', 1.0)
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        
        self.declare_parameter('start_action_for_distance', False)
        self.declare_parameter('start_action_for_angle', False)
        
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.declare_parameter('odom_frame','odom')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        
        
        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y

        self.distance_error = 0
        self.angle_error    = 0

        self.reverse        = 1 # 控制车转动的方向
        self.turn_angle     = 0 # 距离上一次停止后车转动的角度
        self.delta_angle    = 0
        self.real_angle = 0.0
        self.last_real_angle = 0.0
        self.total_angle = 0.0

        self.move_direction = "x"
        self.status_of_finishing_goal = True

        time.sleep(10.0)

        # 创建定时器
        self.work_timer = self.create_timer(0.04, self.timer_work_)

        self.file_path = os.path.expanduser('~/farming_ws/src/motion_controller/config/position_point.yaml')
        self.load_config_file_()
        print ("Finish init work.")
 
    # ---------------- 对外接口函数 ----------------
    def set_distance(self, distance):
        """ 设置车轮方向的行驶距离及以什么样的速度行驶 """
        self.distance = distance

    def set_angle(self, angle):
        """ 设置底盘转动角度 """
        if angle > 0:
            self.angle = angle
        else:
            self.angle = -angle
            self.reverse = -self.reverse

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
    # ----------------------------------------------

    def timer_work_(self):
        # 更新参数
        self.get_param_()
        ref = self.get_odom_angle_()
        self.ori_angle_pid.pid_calculate(ref=ref, goal=self.angle)
        self.move_cmd.angular.z = self.ori_angle_pid.out

        # 距离控制
        if self.start_action_for_distance:
            self.position = self.get_coordinate_value_()
            print("现在位置的 X 坐标: ", self.position.x)
            print("现在位置的 Y 坐标: ", self.position.y)

            o_distance = self.get_O_distance()
            o_distance *= self.odom_linear_scale_correction # 修正
            print("在上一次停下后已经行驶的距离: ", o_distance)

            # 计算误差
            self.distance_error = o_distance - self.distance # 负值控制车向前，正值控制车向后
            print("误差当前值为: ", self.distance_error)

            self.distance_pid.pid_calculate(o_distance, self.distance)
            self.move_cmd.linear.x = self.distance_pid.out

            # if abs(self.distance_error) < self.distance_tolerance: # 达到目标的情况
            #     pass
            #     # 创建了一个名为 start_test 的参数，并将其类型设置为布尔型（BOOL），初始值为 False 。这似乎是为了确保 start_test 参数存在且初始化为 False
            #     # self.start_action_for_distance = rclpy.parameter.Parameter('start_action_for_distance', rclpy.Parameter.Type.BOOL, False)
            #     # all_new_parameters = [self.start_action_for_distance]
            #     # self.set_parameters(all_new_parameters)
            #     # print("任务已完成")
            # else: # 未达到目标的情况
            #     self.distance_pid.pid_calculate(o_distance, self.distance)
            #     self.move_cmd.linear.x = self.distance_pid.out
        else: # 未设定目标的情况
            self.status_of_finishing_goal = False
            self.move_cmd.linear.x = 0.0
            self.x_start = self.get_position_().transform.translation.x
            self.y_start = self.get_position_().transform.translation.y
            print("停车状态下的 X 坐标值: ", self.x_start)
            print("停车状态下的 Y 坐标值: ", self.y_start)
            # self.cmd_vel.publish(Twist())   
        self.cmd_vel.publish(self.move_cmd)

    def get_coordinate_value_(self):
        position = Point()
        position.x = self.get_position_().transform.translation.x
        position.y = self.get_position_().transform.translation.y
        return position

    def get_param_(self):
        self.start_action_for_distance = self.get_parameter('start_action_for_distance').get_parameter_value().bool_value
        self.start_action_for_angle = self.get_parameter('start_action_for_angle').get_parameter_value().bool_value

        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value

        self.distance = self.get_parameter('distance').get_parameter_value().double_value
        self.angle = self.get_parameter('angle').get_parameter_value().double_value
        self.angle = radians(self.angle) #角度转成弧度

        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value

        self.liear_speed = self.get_parameter('liear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

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
            self.real_angle = angle_rot
            if abs(self.real_angle - self.last_real_angle) > 180.0:
                if self.real_angle < self.last_real_angle:
                    self.total_angle += 360.0 - self.last_real_angle + self.real_angle
                else:
                    self.total_angle -= 360.0 - self.real_angle + self.last_real_angle
            else:
                self.total_angle += self.real_angle - self.last_real_angle
            self.last_real_angle = self.real_angle

            return self.total_angle / 180.0 * pi
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


def main():
    rclpy.init()
    try:
        node = Motion_Controller("Motion_Controller")
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
