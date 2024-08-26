import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
import subprocess
import time

class VoiceNotifier(Node):
    def __init__(self):
        super().__init__('voice_notifier')
        # 订阅 Int32MultiArray 类型的消息
        self.subscriber = self.create_subscription(Int32MultiArray, '/voice_commands', self.callback, 10)
        self.voice_state_publisher_ = self.create_publisher(Bool, "/voice_state", 10)
        # 初始化其他变量
        self.place_name = 'A'  # 给一个初始位置
        self.voice_board_params = ['-D', '0', '-d', '0']
        self.voice_state = Bool()
        print('初始化完成')
        # self.data_sort_ = self.dummy_data_sort  # 示例数据排序方法

    def callback(self, msg):
        # 处理收到的 Int32MultiArray 消息
        data = msg.data

        # 验证数据中的所有元素是否都在 [0, 1, 2] 中
        if not all(x in [0, 1, 2] for x in data):
            self.get_logger().warn("Received invalid data in Int32MultiArray.")
            return

        # 获取数据的第一个元素并设置 place_name
        if not data:
            self.get_logger().warn("Data array is empty.")
            return

        first_element = data[0]
        
        if first_element == 0:
            self.place_name = 'A'
        elif first_element == 1:
            self.place_name = 'B'
        elif first_element == 2:
            self.place_name = 'C'
        else:
            self.get_logger().warn("First element of data is not in the valid range [0, 1, 2].")
            return

        # 根据 place_name 选择播报位置
        if self.place_name == 'A':
            self.handle_place_A(data)
        elif self.place_name == 'B':
            self.handle_place_B(data)
        elif self.place_name == 'C':
            self.handle_place_C(data)
    
    def handle_place_A(self, data):
        directions = ['up', 'middle', 'down']
        
        # 从第二个元素开始处理数据
        for index, value in enumerate(data[1:]):  # 使用 data[1:] 从第二个元素开始
            if index >= len(directions):
                self.get_logger().warn("Received more data than expected for place A.")
                break
            
            direction = directions[index]
            voice_type = 'female' if value == 1 else 'male'
            
            self.voice_broadcast(direction)
            self.voice_broadcast(type=voice_type)

    def handle_place_B(self, data):
        # if len(data) > 1:
        print(data)
        self.voice_broadcast(type='female' if data[1] == 1 else 'male')
        # for index, value in enumerate(data[1:]):  # 使用 data[1:] 从第二个元素开始
        #     voice_type = 'female' if value == 1 else 'male'
            
        #     self.voice_broadcast(type=voice_type)

    def handle_place_C(self, data):
        directions = ['left', 'middle', 'right']
        for index, value in enumerate(data[1:]):
            if index >= len(directions):
                self.get_logger().warn("Received more data than expected for place C.")
                break
            self.voice_broadcast(directions[index])
            self.voice_broadcast(type='female' if value == 1 else 'male')
        self.voice_state.data = True
        self.voice_state_publisher_.publish(self.voice_state)

    def voice_broadcast(self, direction='', type=''):
        """ 语音播报 """
        if direction != '':
            if direction == 'up':
                self.guo_xiaoyu_is_broadcasting('The flower above is: ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/up2.wav'] + self.voice_board_params)
                time.sleep(2.5)
            elif direction == 'middle':
                self.guo_xiaoyu_is_broadcasting('The middle flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/middle2.wav'] + self.voice_board_params)
                time.sleep(2.5)
            elif direction == 'down':
                self.guo_xiaoyu_is_broadcasting('The lower flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/down2.wav'] + self.voice_board_params)
                time.sleep(2.5)
            elif direction == 'left':
                self.guo_xiaoyu_is_broadcasting('The left flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/left2.wav'] + self.voice_board_params)
                time.sleep(2.5)
            elif direction == 'right':
                self.guo_xiaoyu_is_broadcasting('The right flower is: ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/right2.wav'] + self.voice_board_params)
                time.sleep(2.5)
        if type != '':
            if type == 'male':
                self.guo_xiaoyu_is_broadcasting('male!!! male!!! male!!! ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/male2.wav'] + self.voice_board_params)
                time.sleep(1.5)
            elif type == 'female':
                self.guo_xiaoyu_is_broadcasting('female!!! female!!! female!!! ')
                subprocess.Popen(['sudo', 'tinyplay', '/root/farming_ws/src/motion_controller/motion_controller/voice/female2.wav'] + self.voice_board_params)
                time.sleep(1.5)

    def guo_xiaoyu_is_broadcasting(self, info):
        self.get_logger().info(info)

    # def dummy_data_sort(self, flowers_lists, prior_axis):
    #     # 示例数据排序方法
    #     return ['famale', 'male', 'famale']

def main(args=None):
    rclpy.init(args=args)
    notifier = VoiceNotifier()
    rclpy.spin(notifier)
    notifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
