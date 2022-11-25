#!/usr/bin/env python
import rospy
import roboclaw_driver as roboclaw
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from math import pi

class Node:

    def __init__(self, mode):
        self.mode = mode
        rospy.init_node("roboclaw_node", anonymous=True)
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "115200"))
        
        self.address = 0X80
	roboclaw.Open(dev_name,baud_rate)
            ################################ SUBSCRIBER ####################################

        rospy.Subscriber("Mode", Int8, self.Mode)
        if self.mode == 0:
            rospy.Subscriber("Udir_track", Twist, self.manualControl)
        elif self.mode == 1:
            rospy.Subscriber("Gestos", Twist, self.controlByGestures)
        elif self.mode ==2:
            rospy.Subscriber("followMe", Twist, self.followMe)
        else:
            pass
            
        rospy.spin()

    def moveForward(self):
        roboclaw.ForwardBackwardM1(self.address, 127)
        roboclaw.ForwardBackwardM2(self.address, 127)

    def moveBackward(self):
        roboclaw.ForwardBackwardM1(self.address, 0)
        roboclaw.ForwardBackwardM2(self.address, 0)

    def rotateLeft(self):
        roboclaw.ForwardBackwardM1(self.address, 127) #Left
        roboclaw.ForwardBackwardM2(self.address, 0)

    def rotateRight(self):
        roboclaw.ForwardBackwardM1(self.address, 0) #Right
        roboclaw.ForwardBackwardM2(self.address, 127)

    def stop(self):
        roboclaw.ForwardBackwardM1(self.address, 63) #Right
        roboclaw.ForwardBackwardM2(self.address, 63)
    
    def moveMotors(self,x,z):

        if x > 0:
            self.moveForward()
            if z > 0:
                self.rotateLeft()
            elif z < 0:
                self.rotateRight()
            else:
                pass
        elif x < 0:
            self.moveBackward()
            if z > 0:
                self.rotateLeft()
            elif z < 0:
                self.rotateRight()
            else:
                pass

        if z > 0:
            self.rotateLeft()
            if x > 0:
                self.moveForward()
            elif x < 0:
                self.moveBackward()
            else:
                pass
        elif z < 0:
            self.rotateRight()
            if x > 0:
                self.moveForward()
            elif x < 0:
                self.moveBackward()
            else:
                pass
	
	if x == 0 and z == 0:
		self.stop()

    def Mode(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.mode = data

    def manualControl(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	x = data.linear.x
	z = data.angular.z
        self.moveMotors(x,z)

    def controlByGestures(self, twist):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	x = data.linear.x
	z = data.angular.z
        self.moveMotors(x,z)

    def followMe(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	x = data.linear.x
	z = data.angular.z
        self.moveMotors(x,z)

if __name__ == '__main__':
    try:
        roboclaw_node = Node(0)
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
