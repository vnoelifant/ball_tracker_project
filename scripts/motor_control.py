#!/usr/bin/env python
import serial
import string
from time import sleep
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class Motion_Control(object):

    def __init__(self, ser):
        #open the serial port
        self.ser = ser

    def send_command(self,list_char):
        data=""
        for c in list_char:
            data+=chr(c)
        self.ser.write(data)
        return

    def degree_to_time(self,degree):
        # 0 degree      == 1000 us
        # 180 degree    == 2000 us
        min_d=0
        max_d=90
        min_t=1000
        max_t=2000

        if degree<min_d or degree>max_d:
            self.ser.close()
            sys.exit()
        else:
            time=(min_t+(max_t-min_t)/(max_d-min_d)*degree)*4
            return time

    def time_to_bytes(self,time):
        n1=time & 0x7F
        n2=(time>>7)&0x7F
        return [n1,n2]

    def set_degree(self,channel_number,degree):
        time = self.degree_to_time(degree)
        num = self.time_to_bytes(time)
        # Compact protocol: 0x84, channel number, target low bits, target high bits
        buff=[0x84,channel_number,num[0],num[1]]
        self.send_command(buff)


def callback(data):
    # Initial global variables
    global ser, comport # Open serial port
    global camerax, cameray # camera degree to control motor
    global centerx, centery # center coordinate of the image

    #Control the two motors
    ballx = data.x
    bally = data.y
    if ballx > centerx and camerax < 89:
        camerax += 2
        comport.set_degree(1, camerax)
    elif ballx < centerx and camerax > 1:
        camerax -= 2
        comport.set_degree(1, camerax)
    if bally > centery and cameray < 89:
        cameray += 2
        comport.set_degree(3, cameray)
    elif bally < centery and cameray > 1:
        cameray -= 2
        comport.set_degree(3, cameray)


ser = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600)
comport = Motion_Control(ser)
camerax = 45
cameray = 45
comport.set_degree(1, camerax)
comport.set_degree(3, cameray)
centerx = 320
centery = 240


def main():

    rospy.init_node('motor_control', anonymous=True)
    rospy.Subscriber('/center_position', Vector3, callback)
    rospy.spin()

    ser.close()


if __name__ == '__main__':
    main()
