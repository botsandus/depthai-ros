#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthImageAnsysNode(Node):

    def __init__(self):
        super().__init__('depth_image_ansys')
        self.subscription = self.create_subscription(
            Image,
            '/oak/stereo/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.image = np.zeros((1280, 800, 3), np.uint8)

    def listener_callback(self, msg):
        # self.get_logger().info("Getting Frame")
        current_frame = self.br.imgmsg_to_cv2(msg)
        # cv2.imshow("oak", current_frame)
        self.get_logger().info(str(
            current_frame[int(msg.height/2)][int(msg.width/2)]))
        # norm = np.linalg.norm(current_frame)
        # current_frame = 500*(current_frame/norm)
        # mean = np.mean((current_frame.flatten()))
        # median = np.median(current_frame.flatten())
        # max = np.max(current_frame.flatten())
        # np.clip(current_frame, 0, mean/2)
        # self.get_logger().info(
        #     str(max)+','
        #     + str(median)+','
        #     + str(mean)
        #     )
        current_frame = 0.5*current_frame
        current_frame = np.clip(current_frame, 0, 255)
        current_frame = np.uint8(current_frame)
        cv2.imshow("oak", cv2.applyColorMap(
            current_frame, cv2.COLORMAP_HOT))
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    try:
        depth_node = DepthImageAnsysNode()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(depth_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            depth_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
