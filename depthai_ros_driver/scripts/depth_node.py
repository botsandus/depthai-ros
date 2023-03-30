#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import depthai as dai


class DepthImageNode(Node):

    def __init__(self):
        super().__init__('depth_image')
        self.image_pub = self.create_publisher(Image, '/depth_image', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.depth_callback)

        self.br = CvBridge()

        # Closer-in minimum depth, disparity range is doubled (from 95 to 190):
        extended_disparity = False
        # Better accuracy for longer distance, fractional disparity 32-levels:
        subpixel = False
        # Better handling for occlusions:
        lr_check = True

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.xout = self.pipeline.create(dai.node.XLinkOut)

        self.xout.setStreamName("disparity")

        # Properties
        self.monoLeft.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.monoRight.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_800_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Create a node that will produce the depth map
        # (using disparity output as it's easier to visualize depth this way)
        self.depth.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        self.depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.depth.setLeftRightCheck(lr_check)
        self.depth.setExtendedDisparity(extended_disparity)
        self.depth.setSubpixel(subpixel)

        # Linking
        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)
        self.depth.disparity.link(self.xout.input)

        with dai.Device(self.pipeline) as device:

            print("Device ID:{}".format(device.getMxId()))
            # Output queue will be used to get the disparity frames
            #  from the outputs defined above
            self.q = device.getOutputQueue(
                name="disparity", maxSize=4, blocking=False)

    # Connect to device and start pipeline

    def depth_callback(self):

        with dai.Device(self.pipeline) as device:

            # inDisparity = self.q.get()
            # # blocking call, will wait until a new data has arrived
            # frame = inDisparity.getFrame()
            # print(frame)
            # try:
            #     self.image_pub.publish(
            #         self.br.cv2_to_imgmsg(frame))
            # except CvBridgeError as e:
            #     print(e)

            # # Normalization for better visualization
            # max_disparity = self.depth.initialConfig.getMaxDisparity()
            # frame = (
            #     frame * (255 / max_disparity)).astype(np.uint8)

            # cv2.imshow("disparity", frame)

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            # frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            # cv2.imshow("disparity_color", frame)


def main(args=None):
    rclpy.init(args=args)

    depth_pub = DepthImageNode()

    rclpy.spin(depth_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
