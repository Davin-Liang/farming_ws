import rclpy
from arm_vision_control import Game_Controller
import time

A_switch                   = True
B_switch                   = True
C_switch                   = True
Home_switch                = False

patrol_speed               = 0.05


def main():
    rclpy.init()
    try:
        node = Game_Controller("Game_Controller")
        node.voice_switch = True
        node.voice_board_params = ['-D', '0', '-d', '0']

        node.only_arm_action        = False       # 只让机械臂到相应的点位，并不进行视觉识别
        node.one_action             = False      # 机械臂到相应的点位，开启视觉，只授粉一朵花
        node.vision_for_voice       = False      # 机械臂到相应的点位，开始视觉，不授粉，只播报

        node.buzzer_tips(times=2.0)

# ---------------------------------------------------------------------------------------------------------------
# ----------------AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA--------------------
# ---------------------------------------------------------------------------------------------------------------
        if A_switch:
            node.add_joint2_pre_slide                    = 8 #TODO:
            node.add_joint3_pre_slide                    = -8 #TODO:
            node.add_joint4_pre_slide                    = -12 #TODO:
            node.add_joint4_slide                        = 9 #TODO:

            for i in range(3):
                node.car_action_in_lidar(patrol_speed-0.015, 0.3, distance_threshold=0.3)
                if i == 0:
                    node.auto_pollinate("A", "a_left", 22000, 1.5, 33000)
                    node.auto_pollinate("A", "a_right", 22000, 1.5, 33000)
                elif i == 1:
                    node.auto_pollinate("A", "a_right", 22000, 1.5, 33000)
                    node.auto_pollinate("A", "a_left", 22000, 1.5, 33000)
                elif i == 2:
                    node.auto_pollinate("A", "a_left", 22000, 1.5, 33000)
                    node.auto_pollinate("A", "a_right", 22000, 1.5, 33000)
            node.set_distance(0.5) # TODO:
            time.sleep(2.0)
            node.set_angle(-90.0, times=5.0)
            node.car_action_in_lidar(-patrol_speed+0.02, 0.6, distance_threshold=0.13)
            node.car_action_in_lidar(-patrol_speed+0.025, 0.6, distance_threshold=0.04)
            time.sleep(1.0)
            node.set_distance(-0.335) #TODO: 
            time.sleep(2.0)
            node.set_angle(0.0, times=5.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB--------------------
# ---------------------------------------------------------------------------------------------------------------
        if B_switch:
            node.add_joint4_pre_slide                    = 4 #TODO:
            node.add_joint3_pre_slide                    = 0 #TODO:
            node.add_joint2_pre_slide                    = 13 #TODO:
            node.add_joint4_slide                        = 0 #TODO:

            node.choose_arm_goal("moving")
            node.car_action_in_lidar(-patrol_speed+0.025, 0.4, distance_threshold=0.15)
            node.set_distance(0.28) #TODO: 
            
            node.auto_pollinate("B", "b_left_front", 150000, 0.3, 30000)
            node.auto_pollinate("B", "b_middle_front", 150000, 0.3, 30000)
            node.auto_pollinate("B", "b_right_front", 150000, 0.3, 30000)
            node.choose_arm_goal("moving")

            for i in range(3):
                node.car_action_in_lidar(-patrol_speed+0.025, 0.4, distance_threshold=0.1)
                node.set_distance(-0.27) #TODO: 
                # arm action
                node.auto_pollinate("B", "b_left_back", 150000, 0.3, 30000)
                node.auto_pollinate("B", "b_middle_back", 150000, 0.3, 30000)
                node.auto_pollinate("B", "b_right_back", 150000, 0.3, 30000)
                
                if i == 2:
                    break
                node.auto_pollinate("B", "b_left_front", 150000, 0.3, 30000)
                node.auto_pollinate("B", "b_middle_front", 150000, 0.3, 30000)
                node.auto_pollinate("B", "b_right_front", 150000, 0.3, 30000)
                node.choose_arm_goal("moving")

            node.set_distance(-0.29) #TODO:0.265
            node.choose_arm_goal("a_left")
            node.set_angle(90.0, times=5.0)
            node.car_action_in_lidar(patrol_speed-0.02, 0.6, distance_threshold=0.05)
            node.car_action_in_lidar(patrol_speed-0.025, 0.6, distance_threshold=0.03)
            node.set_distance(0.335) #TODO: 0.345
            time.sleep(2.0)
            node.set_angle(0.0, times=5.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC--------------------
# ---------------------------------------------------------------------------------------------------------------
        if C_switch:
            node.distance_pid.max_out = 0.0275
            node.vision_for_voice       = True

            node.car_action_in_lidar(0.025, 0.4)
            node.set_distance(0.17)
            node.auto_pollinate("C", "c_left")
            node.auto_pollinate("C", "c_right")
            for i in range(2):
                node.set_distance(0.545) 
                if i == 0:
                    node.auto_pollinate("C", "c_right")
                    node.auto_pollinate("C", "c_left")
                elif i == 1:
                    node.auto_pollinate("C", "c_left")
                    node.auto_pollinate("C", "c_right")
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
