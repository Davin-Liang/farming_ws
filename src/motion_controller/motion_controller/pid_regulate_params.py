import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from math import sqrt, pow, radians
from pid import PID

class Motion_Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        self.ori_angle_pid = PID(Kp=0.685, Ki=0.00079, Kd=0.426, max_out=0.9, max_iout=0.0085)
        self.distance_pid  = PID(Kp=0.42, Ki=0.0, Kd=0.08, max_out=0.85, max_iout=0.0)

        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.move_cmd = Twist()

        self.declare_parameter('Kp_ori', self.ori_angle_pid.Kp)
        self.ori_angle_pid.Kp = self.get_parameter('Kp_ori').get_parameter_value().double_value
        self.declare_parameter('Ki_ori', self.ori_angle_pid.Ki)
        self.ori_angle_pid.Ki = self.get_parameter('Ki_ori').get_parameter_value().double_value
        self.declare_parameter('Kd_ori', self.ori_angle_pid.Kd)
        self.ori_angle_pid.Kd = self.get_parameter('Kd_ori').get_parameter_value().double_value
        self.declare_parameter('max_out_ori', self.ori_angle_pid.max_out)
        self.ori_angle_pid.max_out = self.get_parameter('max_out_ori').get_parameter_value().double_value
        self.declare_parameter('max_iout_ori', self.ori_angle_pid.max_iout)
        self.ori_angle_pid.max_iout = self.get_parameter('max_iout_ori').get_parameter_value().double_value

        self.declare_parameter('Kp_distance', self.distance_pid.Kp)
        self.distance_pid.Kp = self.get_parameter('Kp_distance').get_parameter_value().double_value
        self.declare_parameter('Ki_distance', self.distance_pid.Ki)
        self.distance_pid.Ki = self.get_parameter('Ki_distance').get_parameter_value().double_value
        self.declare_parameter('Kd_distance', self.distance_pid.Kd)
        self.distance_pid.Kd = self.get_parameter('Kd_distance').get_parameter_value().double_value
        self.declare_parameter('max_out_distance', self.distance_pid.max_out)
        self.distance_pid.max_out = self.get_parameter('max_out_distance').get_parameter_value().double_value
        self.declare_parameter('max_iout_distance', self.distance_pid.max_iout)
        self.distance_pid.max_iout = self.get_parameter('max_iout_distance').get_parameter_value().double_value
        
        self.distance = 0.0
        self.set_new_param_to_ros('distance')
        self.angle = 0.0
        self.set_new_param_to_ros('angle')
        self.angle = radians(self.angle)
        
        self.distance_tolerance = 0.03
        self.set_new_param_to_ros('distance_tolerance')
        
        self.odom_linear_scale_correction = 1.0
        self.set_new_param_to_ros('odom_linear_scale_correction')
        
        self.declare_parameter('start_for_pid_distance', False)
        self.declare_parameter('start_for_lidar_distance', False)
        self.start_for_lidar_distance = False
        self.start_for_pid_distance = False

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y

        self.yaw_angle_subcriber_ = self.create_subscription(Float64, "yaw_angle", self.yaw_angle_callback_, 10)
        self.odom_subcriber_ = self.create_subscription(Odometry, "odom", self.odom_callback_, 10)
        self.distance_error  = 0.0
        self.yaw_angle       = 0.0

        # 创建定时器
        self.work_timer = self.create_timer(0.004, self.timer_work_)

        print ("Finish init work.")

    def odom_callback_(self, msg):
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y

    def yaw_angle_callback_(self, msg):
        self.yaw_angle = -msg.data

    def timer_work_(self):
        # 更新参数
        self.get_param_()
        # 姿态控制
        self.ori_angle_pid.pid_calculate(ref=self.yaw_angle, goal=self.angle)
        self.move_cmd.angular.z = self.ori_angle_pid.out

        # 距离控制
        if self.start_for_pid_distance:
            o_distance = self.get_O_distance()
            o_distance *= self.odom_linear_scale_correction # 修正

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
            self.x_start = self.position.x
            self.y_start = self.position.y
            print("正在停车状态......")
        self.cmd_vel.publish(self.move_cmd)

    def get_param_(self):
        self.update_params_from_ros('start_for_pid_distance', 'bool')
        self.update_params_from_ros('odom_linear_scale_correction')
        self.update_params_from_ros('distance')
        self.update_params_from_ros('angle')
        self.angle = radians(self.angle)
        self.update_params_from_ros('distance_tolerance')

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
         
    def get_O_distance(self):
        return sqrt(pow((self.position.x - self.x_start), 2) + pow((self.position.y - self.y_start), 2))       

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
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
