import rclpy
from std_msgs.msg import Int32MultiArray
import serial

LOBOT_CMD_SERVO_MOVE = 3
serialHandle = serial.Serial("/dev/ttyUSB1", 9600)  # 初始化串口， 波特率为9600

def setPWMServoMoveByArray(servos, servos_count, time):
    #控制多个PWM舵机转动
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(servos_count*3+5) #数据长度
    buf.append(LOBOT_CMD_SERVO_MOVE) #指令
    
    servos_count = 1 if servos_count < 1 else servos_count
    servos_count = 254 if servos_count > 254 else servos_count
    buf.append(servos_count) #要控制的舵机个数
    
    time = 0 if time < 0 else time
    time = 30000 if time > 30000 else time
    time_list = list(time.to_bytes(2, 'little'))
    buf.append(time_list[0])    #时间
    buf.append(time_list[1])
    
    for i in range(servos_count):
        buf.append(servos[i*2]) #舵机ID

        pos = servos[i*2+1]
        pos = 500 if pos < 500 else pos
        pos = 2500 if pos > 2500 else pos
        pos_list = list(pos.to_bytes(2, 'little'))
        buf.append(pos_list[0])    #位置
        buf.append(pos_list[1])

    serialHandle.write(buf)


    
def servo_control_callback(msg):
    servo_data = msg.data
    setPWMServoMoveByArray(servo_data, len(servo_data) // 2, 2000)  # 假设时间为1000毫秒


def main():
    rclpy.init()
    node = rclpy.create_node('servo_controller')

    servo_subscriber = node.create_subscription(Int32MultiArray, 'servo_commands', servo_control_callback,10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()