#!/usr/bin/env python3
from typing import List
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from rclpy.parameter import Parameter

# TODO
# Fix pan/cmd to rotate between 90 to -90, starting at 180, 

class PubSubNode(Node):
    def __init__(self):
        super().__init__("pub_sub")
        self.cmd_pub_ = self.create_publisher(Float64, "/cam1/pan_cmd", 10)
        self.target_pose_sub_ = self.create_subscription(Pose, "/model/target/pose", self.target_pose_callback, 10)
        
    def target_pose_callback(self, pose: Pose):
        # Target Position from Callback
        tx = pose.position.x
        ty = pose.position.y
        tz = pose.position.z
        # Camera Position (Hard Coded, WIP)
        cx = 0
        cy = 0
        cz = 1
        
        # Obatin Pan angle
        panAng = np.arctan2((ty-cy), (tx-cx))
        self.get_logger().info(str(panAng))
        msg = Float64()
        msg.data = panAng
        self.cmd_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    rclpy.spin(node)
    rclpy.shutdonw()

if __name__ == "__main__":
    main()
