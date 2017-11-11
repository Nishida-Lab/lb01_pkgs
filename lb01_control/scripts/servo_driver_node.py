#!/usr/bin/python

import rospy
import time
from Adafruit_PWM_Servo_Driver import PWM
from geometry_msgs.msg import Twist


class lb01_servo:
    def __init__(self, channel):
        self.channel = channel
        
    def setSpeed(self, ratio): #-1 < ratio < 1
        if self.channel > 2:
            ratio = -ratio
        if ratio < 0:
            self.speed = (servoMiddle * (1 + ratio)) + 10
            print "[debug]speed = %d" % self.speed
            return int(self.speed)
        elif ratio > 0:
            self.speed = ((servoMax - servoMiddle) * ratio) + servoMiddle
            print "[debug]speed = %d" % self.speed
            return int(self.speed)
        elif ratio == 0:
            self.speed = servoMiddle
            print "[debug]speed = %d" % self.speed
            return int(self.speed)

    def writeServo(self):
        pwm.setPWM(self.channel, 0, int(self.speed))
        
    def __del__(self):
        print "[debug]called destructor"

        
    
# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servoMin = 10 #150  # Min pulse length out of 4096
servoMax = 760 #600  # Max pulse length out of 4096
servoMiddle = (servoMin + servoMax)/2 #( 170 + 600 ) / 2 = 385 

Servo = [0,1,2,3,4,5]

for ch in range(6):
    Servo[ch] = lb01_servo(ch) #generate instance(0~2:right, 3~5:left)

pwm.setPWMFreq(60)   # Set frequency to 60 Hz

ratio = -1.0
print "[debug]ratio = %f" % ratio
for i in range(4):
    ratio = ratio + 0.5
    print "[debug]ratio = %f" % ratio
    for j in range(6):
        Servo[j].setSpeed(ratio)
        Servo[j].writeServo()
    time.sleep(1)

ratio = 0
print "[debug]ratio = %f" % ratio
for k in range(6):
    Servo[k].setSpeed(ratio)
    Servo[k].writeServo()



#def ServoPulse(channel, pulse):
#    pulseLength = 1000000                   # 1,000,000 us per second
#    pulseLength /= 60                       # 60 Hz
#    print "%d us per period" % pulseLength
#    pulseLength /= 4096                     # 12 bits of resolution
#    print "%d us per bit" % pulseLength
#    print "wanabeeeeee"
#    pulse *= 1000
#    pulse /= pulseLength
#    pwm.setPWM(channel, 0, pulse)
