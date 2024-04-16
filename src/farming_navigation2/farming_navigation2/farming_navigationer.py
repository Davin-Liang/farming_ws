#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import json

class Farming_navigationer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Wassup, bro, I am %s, M3!" % name)
        self.action_client = self.create_client(NavigateToPose, 'navigate_to_pose')
        while not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
        self.goal = NavigateToPose.Goal()


    def get_pose_(self, id):
        """ read coordinate for goal point """
        with open("~/farming_robot/farming_ws/src/farming_navigation2/poses/pose_{}.json".format(id), "r") as f:
            text = json.loads(f.read())
            # "position"
            self.pos_x = text["position"]["x"]
            self.pos_y = text["position"]["y"]
            self.pos_z = text["position"]["z"]
            # "orientation"
            self.ori_x = text["orientation"]["x"]
            self.ori_y = text["orientation"]["y"]
            self.ori_z = text["orientation"]["z"]
            self.ori_w = text["orientation"]["w"]

    def send_goal_(self):
        pose = PoseStamped()
        pose.pose.position.x = self.pos_x
        pose.pose.position.y = self.pos_y
        pose.pose.orientation.x = self.ori_x
        pose.pose.orientation.y = self.ori_y
        pose.pose.orientation.z = self.ori_z
        pose.pose.orientation.w = self.ori_w

        self.goal.pose = pose
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback_)
        self.send_goal_future.add_done_callback(self.goal_response_callback_)

    def go_to_point(self, point_id):
        """ A single point navigation is completed by passing in a string of point. """
        self.get_pose_(point_id)
        self.send_goal_()

    def feedback_callback_(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg))

    def goal_response_callback_(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

def main():
    rclpy.init()
    try:
        node = Farming_navigationer('Farming_navigationer')

        node.go_to_point()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
