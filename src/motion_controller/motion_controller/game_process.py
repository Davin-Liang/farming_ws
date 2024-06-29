import rclpy
from farming_visioner import Game_Controller
import time

A_switch                   = False
B_switch                   = False
C_switch                   = False
Home_switch                = False


def main():
    rclpy.init()
    try:
        node = Game_Controller("Game_Controller")
        node.buzzer_tips(times=2.0)
        node.choose_arm_goal('c_right')
        node.vision_control_arm("A", "c_right")
        # for i in range(node.female_num-1):
        #     print(node.female_num)
        #     node.find_next_arm_goal_on_position()
        #     if node.error:
        #         print('重新给次机会')
        #         node.find_next_arm_goal_on_position()
        #         node.error = False
# ---------------------------------------------------------------------------------------------------------------
# ----------------AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA--------------------
# ---------------------------------------------------------------------------------------------------------------
        if A_switch:
            for i in range(3):
                node.start_car_and_lidar_controls_stopping(0.06, 0.4)
                node.vision_control_arm("A", "a_left")
                for i in range(node.female_num-1):
                    print(node.female_num)
                    node.find_next_arm_goal_on_position()
                    if node.error:
                        print('重新给次机会')
                        node.find_next_arm_goal_on_position()
                        node.error = False
                node.vision_control_arm("A", "a_right")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                    if node.error:
                        node.find_next_arm_goal_on_position()
                        node.error = False
            node.set_distance(0.53)
            node.set_angle(-90.0)
            node.start_car_and_lidar_controls_stopping(-0.06, 0.7)
            node.start_car_and_lidar_controls_stopping(-0.06, 0.7)
            node.set_distance(-0.25) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB--------------------
# ---------------------------------------------------------------------------------------------------------------
        if B_switch:
            node.choose_arm_goal("moving")
            node.start_car_and_lidar_controls_stopping(-0.06, 0.4)
            node.set_distance(0.30) #TODO: 距离未确定
            # TODO:机械臂序号未确定
            # node.vision_control_arm("B", "a_left") 
            # node.vision_control_arm("B", "a_left")
            # node.vision_control_arm("B", "a_left")
            node.choose_arm_goal("moving")
            for i in range(3):
                node.start_car_and_lidar_controls_stopping(-0.06, 0.4)
                node.set_distance(-0.27) #TODO: 距离未确定
                # arm action
                # TODO:机械臂序号未确定
                # node.vision_control_arm("B", "a_left") 
                # node.vision_control_arm("B", "a_left")
                # node.vision_control_arm("B", "a_left")
                
                if i == 2:
                    break
                # TODO:机械臂序号未确定 
                # node.vision_control_arm("B", "a_left") 
                # node.vision_control_arm("B", "a_left")
                # node.vision_control_arm("B", "a_left")
                node.choose_arm_goal("moving")

            node.set_distance(-0.33) #TODO: 距离未确定
            node.set_angle(90.0)
            node.start_car_and_lidar_controls_stopping(0.06, 0.6)
            node.start_car_and_lidar_controls_stopping(0.06, 0.6)
            node.set_distance(0.405) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC--------------------
# ---------------------------------------------------------------------------------------------------------------
        if C_switch:
            for i in range(3):
                node.start_car_and_lidar_controls_stopping(0.05, 0.4)
                node.set_distance(0.1) #TODO: 距离未确定
                node.vision_control_arm("C", "a_left")
                for i in range(node.female_num-1):
                    node.find_next_arm_goal_on_position()
                node.vision_control_arm("C", "a_right")
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