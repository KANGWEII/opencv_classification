#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from opencv_classification_interfaces.srv import ObjectDetection

class VisualProcessing(Node):
    def __init__(self):
        super().__init__('visual_processing')
        self.visual_subscriber_ = self.create_subscription(
            Image, "/image_raw", self.visual_callback, 10)
        self.bridge = CvBridge()
        self.detected_object = ""

        self.detection_srv_ = self.create_service(ObjectDetection, 'object_detection', self.detection_callback)

    def visual_callback(self, msg):
        vid = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        vidHSV = cv2.cvtColor(vid, cv2.COLOR_BGR2HSV)

        # Define color ranges
        color_ranges = {
            'orange': ([10, 100, 100], [25, 255, 255]),
            'white': ([0, 0, 50], [180, 50, 255])
        }

        kernel = np.ones((5, 5), "uint8")
        self.detected_object = ""

        for color, (lower, upper) in color_ranges.items():
            lower = np.array(lower, np.uint8)
            upper = np.array(upper, np.uint8)
            mask = cv2.inRange(vidHSV, lower, upper)
            mask = cv2.dilate(mask, kernel)
            res = cv2.bitwise_and(vid, vid, mask=mask)

            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, threshold = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for i, contour in enumerate(contours):
                if i == 0 or cv2.contourArea(contour) < 500:
                    continue

                approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
                cv2.drawContours(vid, [contour], 0, (0, 0, 255), 5)

                M = cv2.moments(contour)
                if M['m00'] != 0:
                    x = int(M['m10'] / M['m00'])
                    y = int(M['m01'] / M['m00'])
                else:
                    x, y = 0, 0

                if len(approx) == 4:
                    shape = 'square'
                elif len(approx) > 4:
                    shape = 'circle'
                else:
                    shape = 'unknown'

                cv2.putText(vid, f'{color} {shape}', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                self.detected_object = f'{color} {shape}'

        cv2.imshow("camera", vid)
        cv2.waitKey(1)

    def detection_callback(self, request, response):
        response.result = f"Detected objects: {self.detected_object}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VisualProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()