#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets
# from std_msgs.msg import String, Bool, Int16MultiArray

class Farming_visioner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        self.area_scaling_factor = 0.5
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)

    def vision_callback_(self, msg):
        # male and female flowers
        flowers_lists = []
        flower = {'Type': '', 'CentralPoint': [], 'Area': 0}
        
        if 0 != len(msg.targets):
            for i in range(len(msg.targets)):
                if msg.targets[i].type == "male": 
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.y_offset + msg.targets[0].rois[0].rect.width/2)
                    flower['Area'] = flower['CentralPoint'][0] * flower['CentralPoint'][1] * self.area_scaling_factor
                elif msg.targets[i].type == "female":
                    flower['Type'] = msg.targets[i].type
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                    flower['CentralPoint'].append(msg.targets[0].rois[0].rect.y_offset + msg.targets[0].rois[0].rect.width/2)
                    flower['Area'] = flower['CentralPoint'][0] * flower['CentralPoint'][1] * self.area_scaling_factor

                flowers_lists.append(flower)

        if 0 != len(flowers_lists):
            pass # TODO:



def main():
    rclpy.init()
    try:
        node = Farming_visioner("Farming_visioner")


        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()