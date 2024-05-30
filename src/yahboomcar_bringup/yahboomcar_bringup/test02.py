import rclpy
from std_msgs.msg import Int32MultiArray

def main():
    rclpy.init()
    node = rclpy.create_node('servo_publisher')

    publisher = node.create_publisher(Int32MultiArray, 'servo_commands', 10)
    msg = Int32MultiArray()

    msg.data = [1, 2000]

    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('Publishing servo commands')
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()