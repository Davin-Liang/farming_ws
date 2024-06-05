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
        self.msg.data = [2000,2000,1500,1500,5000]
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.msg.data[3] < 2200:
            self.msg.data[3] += 50
        else:
            self.msg.data[3] = 1500
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing servo commands')
        print('Current angle of servo 4:', self.msg.data[3])

def main(args=None):
    rclpy.init(args=args)
    servo_publisher = ServoPublisher()
    rclpy.spin(servo_publisher)
    servo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()