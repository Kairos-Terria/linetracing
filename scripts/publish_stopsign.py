#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from img_utils import TrafficInfo


class StopSign:
    def __init__(self):
        self.traffic_info = TrafficInfo()
        
        self.image_sub = rospy.Subscriber('/myagv/image_raw', Image, self.sub_image)
        self.stop_sign_pub = rospy.Publisher('/myagv/stop_sign', Bool, queue_size=10)

    def sub_image(self, data):
        try:
            image = CvBridge().imgmsg_to_cv2(data, "bgr8")
            stop_sign = self.traffic_info.get_go_stop_cv2(image)
            self.stop_sign_pub.publish(stop_sign)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__=='__main__':
    try:
        rospy.init_node('publish_stopsign', anonymous=False)

        StopSign()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
