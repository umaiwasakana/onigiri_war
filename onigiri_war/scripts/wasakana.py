#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time

# from abstractRulo import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class RandomBot():
    def __init__(self):

        self.state = 'go' 
        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.r, self.l = 0,0
        self.muki = 0.5

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def jointstateCallback(self, data):
        '''
        update wheel rotation num
        '''
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]


    def calcState(self):

        kaiten = abs(self.wheel_rot_r - self.wheel_rot_l)

        print (kaiten, self.wheel_rot_r, self.wheel_rot_l)
        if self.state == 'go' and self.wheel_rot_r >= 33:
            print ("finish:", self.state)
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            print (self.r)
            self.state = 'migi'

        elif self.state == 'migi' and self.wheel_rot_r >= (self.r+5.5):# and self.wheel_rot_l >= (self.l - 5.5):
            print ("finish:", self.state)
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'go2'

        elif self.state == 'go2' and self.wheel_rot_r > (self.r+65):
            print ("finish:", self.state)
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'hidari'

        elif self.state == 'hidari' and self.wheel_rot_r <= (self.r-2.1) and self.wheel_rot_l >= (self.r+2.2):
            print ("finish:", self.state)
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'go3'

        elif self.state == 'go3' and self.wheel_rot_r > (self.r+35):
            print ("finish:", self.state)
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'hidari1'

        elif self.state == 'hidari1' and self.wheel_rot_l >= (self.l+5):
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'go4'

        elif self.state == 'go4' and self.wheel_rot_r >= (self.r+10):
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'hidari2'

        elif self.state == 'hidari2' and self.wheel_rot_l >= (self.l+11):
            self.r, self.l = self.wheel_rot_r, self.wheel_rot_l
            self.state = 'stop'

        #elif self.state == 'hidari' and kaiten > 10 and self.wheel_rot_l > 10:
        #    self.state = 'migi'

    def calcTwist(self):

        if self.state == 'hidari' or self.state == 'hidari1' or self.state == 'hidari2':
            x = 0
            th = self.muki
        elif self.state == 'migi' or self.state == 'migi1':
            x = 0
            th = -1 * self.muki
        elif self.state == 'go' or self.state == 'go2' or self.state == 'go3' or self.state == 'go4':
            # set speed x axis
            x = self.muki
            th = 0
        else:
            x = 0
            th = 0
            # rospy.logerr("SioBot state is invalid value %s", self.state)

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):

        r = rospy.Rate(5) # change speed 1fps

        while not rospy.is_shutdown():
            # update state from now state and wheel rotation
            self.calcState()
            # update twist
            twist = self.calcTwist()

            # publish twist topic
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot()
    bot.strategy()


