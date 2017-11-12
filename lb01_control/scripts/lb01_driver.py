#!/usr/bin/python
import rospy
import time
from Adafruit_PWM_Servo_Driver import PWM
from geometry_msgs.msg import Twist


class LB01Driver:
    def __init__(self):
        self.right_channel = [0,1,2]
        self.left_channel = [3,4,5]
        self.servoMin = 10 # Min pulse length out of 4096
        self.servoMax = 760 # Max pulse length out of 4096
        self.servoMiddle = (self.servoMin + self.servoMax)/2.
        self.tread = rospy.get_param('~tread', 0.19)
        self.left_radius = rospy.get_param('~left_radius', 0.05)
        self.right_radius = rospy.get_param('~right_radius', 0.05)
        self.max_rad_ps = 3.8785 # 222.22222 [deg]
        self.max_right_vel = self.max_rad_ps * self.right_radius
        self.max_left_vel = self.max_rad_ps * self.left_radius
        self.max_linear_vel = (self.max_right_vel + self.max_left_vel)/ 2.
        self.max_angular_vel = (self.max_right_vel - self.max_left_vel)/self.tread

        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.CmdVelCallback)

        # Initialise the PWM device using the default address
        pwm = PWM(0x40)
        pwm.setPWMFreq(60)   # Set frequency to 60 Hz

    def CmdVelCallback(self, msg):
        self.Drive(msg)

    def Drive(self, cmd_vel):
        # if larger than 
        if abs(cmd_vel.linear.x) > abs(self.max_linear_vel):
            cmd_vel.linear.x = cmd_vel.linear.x/abs(cmd_vel.linear.x) * self.max_linear_vel
        if abs(cmd_vel.angular.z) > abs(self.max_angular_vel):
            cmd_vel.angular.z = cmd_vel.angular.z/abs(cmd_vel.angular.z) * self.max_angular_vel

        # Culculate each velocity
        target_right_vel = cmd_vel.linear.x + (self.tread/2.)*cmd_vel.angular.z
        target_left_vel = cmd_vel.linear.x + (self.tread/2.)*cmd_vel.angular.z

        # Culculate each ratio
        right_vel_ratio = (target_right_vel + self.max_right_vel) / (2.0*self.max_right_vel)
        left_vel_ratio = (-target_left_vel + self.max_left_vel) / (2.0*self.max_left_vel)

        # Culculate each pulse
        right_pulse = self.getPulse(right_vel_ratio)
        left_pulse = self.getPulse(left_vel_ratio)

        # Write PWM pulse
        for i in self.right_channel:
            pwm.setPWM(i, 0, right_pulse)
        for i in self.left_channel:
            pwm.setPWM(i, 0, left_pulse)
        
    def getPulse(self, ratio):
        pulse = (self.servoMax - self.servoMin) * ratio + self.servoMin
        return int(pulse)
                
    def __del__(self):
        print "[debug]called destructor"


if __name__ == '__main__':
    rospy.init_node("lb01_driver")
    lb01_driver = LB01Driver()
    rospy.spin()
