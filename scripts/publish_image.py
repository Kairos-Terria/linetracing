#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_image():
    image_pub = rospy.Publisher('/myagv/image_raw', Image, queue_size=10)
    rospy.init_node('publish_image', anonymous=False)

    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            image_pub.publish(CvBridge().cv2_to_imgmsg(frame, "bgr8"))

if __name__=='__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
    
