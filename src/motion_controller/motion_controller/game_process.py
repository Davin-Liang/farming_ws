import rclpy
from rclpy.node import Node
from farming_visioner import Farming_visioner
from .motion_controller import Motion_Controller

def main():
    rclpy.init()
    try:
        FarmingVisioner = Farming_visioner("Farming_visioner")
        MotionController = Motion_Controller("Motion_Controller")

        MotionController.set_distance(1.0)
        FarmingVisioner.vision_control_arm('a_left')
    

    except KeyboardInterrupt:
        pass
    finally:
        if FarmingVisioner and MotionController:
            FarmingVisioner.destroy_node()
            MotionController.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()