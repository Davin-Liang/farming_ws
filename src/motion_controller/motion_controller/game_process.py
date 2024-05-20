import rclpy
from rclpy.node import Node
from farming_visioner import Farming_visioner
from motion_controller import Motion_Controller
import time

def main():
    rclpy.init()
    try:
        FV = Farming_visioner("Farming_visioner")

        # MC = Motion_Controller("Motion_Controller")

        # MC.set_distance(-1.0)
        # MC.set_angle(-90.0)
        # MC.start_car_and_lidar_controls_stopping(0.05, 0.4, 1)
        print("111111")
        FV.vision_control_arm('a_left')
        # time.sleep(1.0)
        # FV.find_next_arm_goal_on_position()
        # MC.set_distance(1.0)
        # MC.set_angle(90.0)
    
        while 1:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        if FV:
            FV.destroy_node()
            # MC.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()