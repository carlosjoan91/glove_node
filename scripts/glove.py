#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import JointState


def convert_range(number):
	if number > 600: #Fully closed
		n = 600
	elif number < 350:
		n = 350 #Fully open
	else n = number
	n = 600 - n ##Change value to the 0-250 range and invert its direction (it now grows as you close the fist)
	return n * (3.0/250.0) ##Change the value to the 0.0 to 3.0 range to make it similar to the robot's hands range

def glovenode():
	unid = 1
	ser1 = serial.Serial('/dev/ttyUSB0') ##TODO: Make this a parameter/argument probably
	ser2 = serial.Serial('/dev/ttyUSB1') ##TODO: Wait for the serial device instead of erroring out
	rospy.init_node('glove_node', anonymous=True)
	rate = rospy.Rate(100)
	pub = rospy.Publisher('/glove_status', JointState, queue_size=1)
	
	while not rospy.is_shutdown():
		lastline = (ser1.readline()).split(',')
		if lastline[0] == 'L':
			left_value_list = [convert_range(int(x)) for x in lastline[1:]]
		elif lastline[0] == 'R':
			right_value_list = [convert_range(int(x)) for x in lastline[1:]]

		lastline = (ser2.readline()).split(',')
		if lastline[0] == 'L':
			left_value_list = [convert_range(int(x)) for x in lastline[1:]]
		elif lastline[0] == 'R':
			right_value_list = [convert_range(int(x)) for x in lastline[1:]]
		
		msg = JointState()
		msg.header.stamp = rospy.Time.now()
		msg.name = ['leftPinky', 'leftRing', 'leftMiddle', 'leftIndex', 'leftThumb', 'rightPinky', 'rightRing', 'rightMiddle', 'rightIndex', 'rightThumb']
		msg.position = left_value_list + right_value_list

		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
    try:
        glovenode()
    except rospy.ROSInterruptException:
        pass