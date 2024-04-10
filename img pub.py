#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_message():
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    rospy.init_node('video_pub_py', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cap = cv2.imread('/home/kqubes/Downloads/tree-736885_1280.jpg', cv2.IMREAD_GRAYSCALE)  # Read in grayscale
    br = CvBridge()

    while not rospy.is_shutdown():
        rospy.loginfo('Publishing video frame')
        pub.publish(br.cv2_to_imgmsg(cap, encoding="mono8"))  # Convert to mono8 encoding
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
