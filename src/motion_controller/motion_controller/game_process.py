import rclpy
from farming_visioner import Game_Controller
import time

A_switch                   = True
B_switch                   = False
C_switch                   = False
Home_switch                = False


def main():
    rclpy.init()
    try:
        node = Game_Controller("Game_Controller")

# ---------------------------------------------------------------------------------------------------------------
# ----------------AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA--------------------
# ---------------------------------------------------------------------------------------------------------------
        if A_switch:
            for i in range(3):
                node.start_car_and_lidar_controls_stopping(0.05, 0.4)
                node.vision_control_arm("A","a_left")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.vision_control_arm("A","a_right")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
            node.set_distance(0.8) # TODO: 距离未确定
            node.set_angle(-90.0)
            node.start_car_and_lidar_controls_stopping(-0.05, 0.6)
            node.start_car_and_lidar_controls_stopping(-0.05, 0.6)
            node.set_distance(-0.35) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB--------------------
# ---------------------------------------------------------------------------------------------------------------
        if B_switch:
            node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
            node.set_distance(0.15) #TODO: 距离未确定
            # TODO:机械臂序号未确定
            node.vision_control_arm("B","a_left") 
            node.vision_control_arm("B","a_left")
            node.vision_control_arm("B","a_left")
            node.choose_arm_goal("a_left")
            for i in range(3):
                node.start_car_and_lidar_controls_stopping(-0.05, 0.4)
                node.set_distance(-0.15) #TODO: 距离未确定
                # arm action
                # TODO:机械臂序号未确定
                node.vision_control_arm("B","a_left") 
                node.vision_control_arm("B","a_left")
                node.vision_control_arm("B","a_left")
                
                if i == 2:
                    break
                # TODO:机械臂序号未确定 
                node.vision_control_arm("B","a_left") 
                node.vision_control_arm("B","a_left")
                node.vision_control_arm("B","a_left")
                node.choose_arm_goal("a_left")

            node.set_distance(-0.2) #TODO: 距离未确定
            node.set_angle(90.0)
            node.start_car_and_lidar_controls_stopping(0.05, 0.6)
            node.start_car_and_lidar_controls_stopping(0.05, 0.6)
            node.set_distance(0.25) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC--------------------
# ---------------------------------------------------------------------------------------------------------------
        if C_switch:
            for i in range(3):
                node.start_car_and_lidar_controls_stopping(0.05, 0.4)
                node.set_distance(0.1) #TODO: 距离未确定
                node.vision_control_arm("C","a_left")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.vision_control_arm("C","a_right")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.start_car_and_lidar_controls_stopping(0.05, 0.4, 0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH--------------------
# ---------------------------------------------------------------------------------------------------------------
        if Home_switch:
            pass
# ---------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------

        while 1:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()