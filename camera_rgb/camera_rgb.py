#!/usr/bin/env python3
import time
import yaml
import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo

class CameraPublisher(Node):
    def __init__(self, source=0, width=640, height=480, fps=30):
        super().__init__("camera_publisher")

        self.publisher_ = self.create_publisher(Image, "camera/image", 100)
        self.publisher_info = self.create_publisher(CameraInfo, "camera/camera_info", 100)
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps

        self.image_publisher()


    def info_set(self):
        data1 = [679.0077123045467, 0.0, 356.3515350783442, 0.0, 672.9969017826554, 196.5430429125135, 0.0, 0.0, 1.0]
        data2 = [0.260086, -0.025048, 0.089063, 0.138628, 0.000000]
        data3 = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
        data4 = [852.395142, 0.000000, 565.897630, 0.000000, 0.000000, 922.066223, 386.586250, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        people = {'image_width': 640,
          'image_height': 480,
          'camera_name': "camera",
          "camera_matrix": {'rows': 3, 'cols': 3, 'data': data1},
          "distortion_model": "plumb_bob",
          "distortion_coefficients": {'rows': 1, 'cols': 5, 'data': data2},
          "rectification_matrix": {'rows': 3, 'cols': 3, 'data': data3},
          "projection_matrix": {'rows': 3, 'cols': 4, 'data': data4}}

        msg = CameraInfo()
        msg.header.frame_id = people["camera_name"]
        msg.height = people["image_height"]
        msg.width = people["image_width"]
        msg.distortion_model = people["distortion_model"]
        msg.d = people["distortion_coefficients"]["data"]
        msg.k = people["camera_matrix"]["data"]
        msg.r = people["rectification_matrix"]["data"]
        msg.p = people["projection_matrix"]["data"]

        self.publisher_info.publish(msg)
        # with open(r'test.yaml', 'w+') as f:
        #     y = ""
        #     for x in str(yaml.dump(people, sort_keys=False, default_flow_style=False)):
        #         if x == "'":
        #             pass
        #         else:
        #             y = y + x
        #     f.write(y)
    def image_publisher(self):
        cap = cv.VideoCapture(self.source)
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G')) # depends on fourcc available camera
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv.CAP_PROP_FPS, self.fps)

        bridge = CvBridge()
        frame_id_ = 1

        mtx = [633.9817971, 0.0, 319.34040276, 0.0, 632.24890996, 196.56498982, 0.0, 0.0, 1.0]
        mtx = np.array(mtx)
        mtx = mtx.reshape((3,3))
        dist = [-0.42858939, 0.22824651, 0.00167929, 0.0033874, -0.11409583]
        dist = np.array(dist)
        while True:
            ret, frame = cap.read()
            
            if ret:
                try:
                    h,  w = frame.shape[:2]
                    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
                    frame = cv.undistort(frame, mtx, dist, None, newcameramtx)
                    msg = bridge.cv2_to_imgmsg(np.array(frame), encoding="bgr8")
                    msg.header.stamp.sec = int(time.time())
                    msg.header.frame_id = "camera"
                    self.publisher_.publish(msg)
                    self.info_set()
                except CvBridgeError as e:
                    print(e)
                frame_id_ = frame_id_ + 1
            
            if cv.waitKey(1) & 0xFF == ord("q"):
                break

            


def main():
    rclpy.init()

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass

    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


            


