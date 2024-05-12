import rclpy
from rclpy.node import Node
from farming_visioner import Farming_visioner
from .motion_controller import Motion_Controller
import time

def main():
    rclpy.init()
    try:
        FarmingVisioner = Farming_visioner("Farming_visioner")
        MotionController = Motion_Controller("Motion_Controller")

        MotionController.set_distance(1.0)
        FarmingVisioner.vision_control_arm('a_left')
        time.sleep(1.0)
        FarmingVisioner.find_next_arm_goal_on_position()
        MotionController.set_distance(1.0)
        MotionController.set_angle(90.0)
    

    except KeyboardInterrupt:
        pass
    finally:
        if FarmingVisioner and MotionController:
            FarmingVisioner.destroy_node()
            MotionController.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()