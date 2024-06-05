import rclpy
from std_msgs.msg import Int32MultiArray

def main():
    rclpy.init()
    node = rclpy.create_node('servo_publisher')

    publisher = node.create_publisher(Int32MultiArray, 'servo_commands', 10)
    msg = Int32MultiArray()

    msg.data = [2000,2000,1500,1500,5000]  #前四位为舵机对应角度，最后一位为时间

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
    rclpy.shutdown()

if __name__ == '__main__':
    main()