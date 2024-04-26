#!/usr/bin/env python3

import rospy
import cv2

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from img_utils import get_direction


class Direction:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/myagv/image_raw', Image, self.sub_image)
        self.direction_pub = rospy.Publisher('/myagv/direction', Float32, queue_size=10)

    def sub_image(self, data):
        try:
            image = CvBridge().imgmsg_to_cv2(data, "bgr8")
            direction = get_direction(image)
            self.direction_pub.publish(direction)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__=='__main__':
    try:
        rospy.init_node('publish_direction', anonymous=False)

        Direction()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
