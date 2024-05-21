import rclpy
from rclpy.node import Node
from farming_visioner import Farming_visioner
from motion_controller import Motion_Controller
from rclpy.executors import MultiThreadedExecutor
import time
from  threading import Thread

def sspin(a):
    a.spin()

def main():
    rclpy.init()
    try:
        FV = Farming_visioner("Farming_visioner")


        MC = Motion_Controller("Motion_Controller")

        try:
            # Spin up the nodes
            rclpy.spin(FV)
            print("111111111")
            rclpy.spin(MC)
        except KeyboardInterrupt:
            pass
        # executor = MultiThreadedExecutor(num_threads=2)
        # executor.add_node(FV)
        # executor.add_node(MC)
        # executor.spin()
        # print("222222222222")
        # time.sleep(2.0)
        # spin_thread = Thread(target=sspin, args=(executor, ))
        # spin_thread.start()
        # MC.set_distance(-1.0)
        # MC.set_angle(-90.0)
        MC.start_car_and_lidar_controls_stopping(0.05, 0.4)
        # print("111111")


        # FV.vision_choose_goal_in_A("a_left")
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