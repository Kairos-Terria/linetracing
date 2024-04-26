#!/usr/bin/env python3

from pymycobot.myagv import MyAgv
import time
import rospy
from std_msgs.msg import Float32, Bool
import threading

agv = MyAgv("/dev/ttyAMA2", 115200)

class move:
    def __init__(self):
        self.obstacle = None
        self.direction = None
        self.stop_sign = None

        self.obstacle_sub = rospy.Subscriber('/obstacle', Bool, self.obstacle_callback)
        self.direction_sub = rospy.Subscriber('/myagv/direction', Float32, self.direction_callback)
        self.stop_sign_sub = rospy.Subscriber('/myagv/stop_sign', Bool, self.stop_sign_callback)

    def obstacle_callback(self, data):
        self.obstacle = data.data

    def direction_callback(self, data):
        self.direction = data.data

    def stop_sign_callback(self, data):
        self.stop_sign = data.data
    
    def move(self):

        while True:
            if self.direction == 2:  # LEFT
                agv.go_ahead(1)
                agv.counterclockwise_rotation(1)
                print("LEFT")
        
            elif self.direction == 3:  # RIGHT
                agv.go_ahead(1)
                agv.clockwise_rotation(1)
                print("RIGHT")
            
            elif self.direction == 1 :  # Go
                agv.go_ahead(1)
                print("GO")

            elif self.direction == 4:
                agv.stop()
                time.sleep(0.1)
                print("STOP: no way")

                t = time.time()
                while self.direction == 4 and time.time() - t < 1.5:
                    agv.clockwise_rotation(1)

                agv.stop()
                time.sleep(0.1)

                t = time.time()
                while self.direction == 4 and time.time() - t < 2:
                    agv.counterclockwise_rotation(1)

                agv.stop()
                time.sleep(1)

                if self.direction == 4:
                    print("done")
                    agv.stop()
                    time.sleep(1)

            if self.stop_sign:
                print("STOP: red light")
                agv.stop()
                time.sleep(2)
            
            if self.obstacle:
                agv.stop()
                time.sleep(2)
                print("obstacle!!!")


    
        

if __name__ == '__main__':

    try:
        rospy.init_node('move_agv', anonymous=False)

        m = move()

        t1 = threading.Thread(target=m.move, daemon=True)
        t1.start()

        while not rospy.is_shutdown():
            rospy.spin()

        agv.stop()

    except rospy.ROSInterruptException:
        pass
