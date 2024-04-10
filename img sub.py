#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('video_sub_py', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber('video_frames', Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Video Window", cv_image)
            cv2.waitKey(0)  # Wait for 1 millisecond
         
        except Exception as e:
            print(e)

def main():
    try:
        image_subscriber = ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
