import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile
from rclpy.timer import Timer

class ServoPublisher(Node):
    def __init__(self):
        super().__init__('servo_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_commands', 10)
        self.msg = Int32MultiArray()
        self.msg.data = [1240,1700,1700,1500,2000]
        self.timer_ = self.create_timer(2.5, self.timer_callback)

    def timer_callback(self):
        if self.msg.data[3] < 2200:
            self.msg.data[3] += 50
        else:
            self.msg.data[3] = 1500
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing servo commands')
        print('Current angle of servo 4:', self.msg.data[3])

<<<<<<< HEAD
    msg.data = [2000,2000,1500,1500,5000]  #前四位为舵机对应角度，最后一位为时间
=======
<<<<<<< HEAD
def main(args=None):
    rclpy.init(args=args)
    servo_publisher = ServoPublisher()
    rclpy.spin(servo_publisher)
    servo_publisher.destroy_node()
=======
    msg.data = [500,500,500,500,2000]  #前四位为舵机对应角度，最后一位为时间
>>>>>>> cbd84c7a7652ef06b964588dcc94bf63e9260d71

    while rclpy.ok():
        if msg.data[3] < 2200:  # 如果舵机四的角度小于2200，增加50
            msg.data[3] += 50
        else:                   # 若大于等于2200，重置为1500
            msg.data[3] = 1500
        publisher.publish(msg)
        node.get_logger().info('Publishing servo commands')
        print('Current angle of servo 4:', msg.data[3])
        rclpy.spin_once(node)

    node.destroy_node()
>>>>>>> 9a2bba387580006ffea68c9c1b430a48ef6f3252
    rclpy.shutdown()

if __name__ == '__main__':
    main()