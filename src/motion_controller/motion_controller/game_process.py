import rclpy
from farming_visioner import Game_Controller
import time

A_switch                   = True
B_switch                   = False
C_switch                   = False
Home_switch                = False

patrol_speed               = 0.06


def main():
    rclpy.init()
    try:
        node = Game_Controller("Game_Controller")
        node.voice_switch = True
        node.voice_board_params = ['-D', '0', '-d', '0']

        node.only_arm_action        = False      # 只让机械臂到相应的点位，并不进行视觉识别
        node.one_action             = False      # 机械臂到相应的点位，开启视觉，只授粉一朵花
        node.vision_for_voice       = False     # 机械臂到相应的点位，开始视觉，不授粉，只播报

        node.buzzer_tips(times=2.0)
        node.choose_arm_goal("a_left")

        #node.auto_pollinate("A", "a_left")
        #node.auto_pollinate("A", "a_right")

# ---------------------------------------------------------------------------------------------------------------
# ----------------AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA--------------------
# ---------------------------------------------------------------------------------------------------------------
        if A_switch:
            for i in range(3):
                node.car_action_in_lidar(patrol_speed, 0.4)
                node.auto_pollinate("A", "a_left")
                node.auto_pollinate("A", "a_right")
            node.set_distance(0.55)
            node.set_angle(-90.0)
            node.car_action_in_lidar(-patrol_speed, 0.7)
            node.car_action_in_lidar(-patrol_speed, 0.7)
            node.set_distance(-0.27) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB--------------------
# ---------------------------------------------------------------------------------------------------------------
        if B_switch:
            node.choose_arm_goal("moving")
            node.car_action_in_lidar(-patrol_speed, 0.4)
            node.set_distance(0.30) #TODO: 距离未确定
            # TODO:机械臂序号未确定
            node.auto_pollinate("B", "b_left_front")
            node.auto_pollinate("B", "b_middle_front")
            node.auto_pollinate("B", "b_right_front")
            node.choose_arm_goal("moving")

            for i in range(3):
                node.car_action_in_lidar(-patrol_speed, 0.4)
                node.set_distance(-0.25) #TODO: 距离未确定
                # arm action
                # TODO:机械臂序号未确定
                node.auto_pollinate("B", "b_left_back")
                node.auto_pollinate("B", "b_middle_back")
                node.auto_pollinate("B", "b_right_back")
                
                if i == 2:
                    break
                # TODO:机械臂序号未确定 
                node.auto_pollinate("B", "b_left_front")
                node.auto_pollinate("B", "b_middle_front")
                node.auto_pollinate("B", "b_right_front")
                node.choose_arm_goal("moving")

            node.set_distance(-0.33) #TODO: 距离未确定
            node.set_angle(90.0)
            node.car_action_in_lidar(patrol_speed, 0.6)
            node.car_action_in_lidar(patrol_speed, 0.6)
            node.set_distance(0.425) #TODO: 距离未确定
            node.set_angle(0.0)
# ---------------------------------------------------------------------------------------------------------------
# ----------------CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC--------------------
# ---------------------------------------------------------------------------------------------------------------
        if C_switch:
            node.car_action_in_lidar(0.05, 0.4)
            node.set_distance(0.1)
            for i in range(3):
                node.set_distance(0.25) #TODO: 距离未确定
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
