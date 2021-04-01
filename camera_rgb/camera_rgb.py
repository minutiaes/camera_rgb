#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_raw = self.create_publisher(Image, "camera/image_color", 5)
        self.publisher_rect = self.create_publisher(Image, "camera/image_undist_color", 5)
        self.publisher_info = self.create_publisher(CameraInfo, "camera/camera_info", 10)

        self.cam_info = self.read_config()
        self.width = self.cam_info["image_width"]
        self.height = self.cam_info["image_height"]
        self.source = self.cam_info["camera_source"]
        self.fps = self.cam_info["camera_fps"]
        self.name = self.cam_info["camera_name"]
        self.mtx = np.array(self.cam_info["camera_matrix"]["data"]).reshape((3, 3))
        self.dist_model = self.cam_info["distortion_model"]
        self.dist = np.array(self.cam_info["distortion_coefficients"]["data"])

        self.msg_cam_info = self.msg_set()
        self.declare_parameters(
            namespace="",
            parameters=[
                ("read_video", None),
                ("video_path", None),
            ]
        )
        self.read_video = self.get_parameter(name="read_video").get_parameter_value().bool_value
        self.video_path = self.get_parameter(name="video_path").get_parameter_value().string_value
        self.image_publisher()



    @classmethod
    def read_config(cls):
        """reads YAML file"""
        config_file = os.path.join(
            get_package_share_directory('camera_rgb'), 'config', 'camera.yaml')
        with open(config_file, 'r') as f:
            return yaml.load(f, Loader=yaml.FullLoader)


    def msg_set(self):
        """prepares CameraInfo msg"""
        msg = CameraInfo()
        msg.header.frame_id = self.name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.height
        msg.width = self.width
        msg.distortion_model = self.dist_model
        msg.d = self.cam_info["distortion_coefficients"]["data"]
        msg.k = self.cam_info["camera_matrix"]["data"]

        return msg

    def image_publisher(self):
        if not self.read_video:
            # from camera
            cap = cv.VideoCapture(self.source)
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv.CAP_PROP_FPS, self.fps)

            newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width, self.height), 1, (self.width, self.height))
            mapx, mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (self.width, self.height), 5)
        elif self.read_video:
            # from video
            cap = cv.VideoCapture(self.video_path)
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M','P','4','2'))
        if not cap.isOpened():
            self.get_logger().info(f"camera or video file couldn't be opened!")
        bridge = CvBridge()
        while True:
            ret, frame = cap.read()

            if ret:
                try:
                    if not self.read_video:
                        # undistortion
                        frame_rect = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)
                    elif self.read_video:
                        frame_rect = frame
                    msg = bridge.cv2_to_imgmsg(np.array(frame_rect), "bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.msg_cam_info.header.stamp = msg.header.stamp
                    msg.header.frame_id = "camera"
                    self.publisher_rect.publish(msg)
                    msg_ = bridge.cv2_to_imgmsg(np.array(frame), "bgr8")
                    msg.data = msg_.data
                    self.publisher_raw.publish(msg)
                    self.publisher_info.publish(self.msg_cam_info)
                except CvBridgeError as err:
                    print(err)


def main():
    rclpy.init()

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
