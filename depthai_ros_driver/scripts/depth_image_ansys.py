#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


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

    def listener_callback(self, msg):
        # self.get_logger().info("Getting Frame")
        current_frame = self.br.imgmsg_to_cv2(msg)
        self.get_logger().info(str(current_frame.shape))
        # cv2.imshow("oak", current_frame)
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
