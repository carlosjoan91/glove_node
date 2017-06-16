#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int32
from numl_val_msgs.msg import HandPoseTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

def glovenode():
	unid = 1
	ser = serial.Serial('/dev/ttyUSB0')
	rospy.init_node('glove_node', anonymous=True)
	rate = rospy.Rate(100)
	pub = rospy.Publisher('/arm_control', HandPoseTrajectoryRosMessage, queue_size=1)
	msg = HandPoseTrajectoryRosMessage()
	msg.robot_side = 1
	msg.execution_mode = 0
	msg.desired_pose = 0
	msg.previous_message_id = unid - 1
	msg.homeAllForearmJoints = True
	troll = OneDoFJointTrajectoryRosMessage()
	tprox = OneDoFJointTrajectoryRosMessage()
	tdist = OneDoFJointTrajectoryRosMessage()
	index = OneDoFJointTrajectoryRosMessage()
	middle = OneDoFJointTrajectoryRosMessage()
	pinky = OneDoFJointTrajectoryRosMessage()
	fingers_list = [troll, tprox, tdist, index, middle, pinky]
	msg.hand_joint_trajectory_messages = fingers_list
	null_point1d = TrajectoryPoint1DRosMessage()
	null_point1d.position = 0.5

	null_onedof_1 = OneDoFJointTrajectoryRosMessage()
	null_onedof_2 = OneDoFJointTrajectoryRosMessage()
	null_onedof_3 = OneDoFJointTrajectoryRosMessage()
	msg.forearm_joint_trajectory_messages = [null_onedof_1, null_onedof_2, null_onedof_3]

	for finger in fingers_list:
		finger.trajectory_points.append(null_point1d)
	while not rospy.is_shutdown():
		msg.unique_id = unid
		lastline = ser.readline()
		value_list = [int(x) for x in lastline.split(',')]
		clamped_values = []
		for val in value_list:
			new_val = val
			if val < 350:
				new_val = 350
			if val > 600:
				new_val = 600
			new_val -= 350
			new_val = 250 - new_val
			clamped_values.append(new_val)
		print clamped_values
		rin = clamped_values[0]
		mid = clamped_values[1]
		ind = clamped_values[2]
		thumb = clamped_values[3]
		#print thumb
		rin = (rin * 2.7/250) + 0.3
		mid = (mid * 2.7/250) + 0.3
		ind = (ind * 2.7/250) + 0.3
		thumb = (thumb * 1.7/250) + 0.3

		pinky_point1d = TrajectoryPoint1DRosMessage()
		pinky_point1d.position = rin
		pinky.trajectory_points = [pinky_point1d] 

		middle_point1d = TrajectoryPoint1DRosMessage()
		middle_point1d.position = mid
		middle.trajectory_points = [middle_point1d] 

		index_point1d = TrajectoryPoint1DRosMessage()
		index_point1d.position = ind
		index.trajectory_points = [index_point1d] 

		thumb_point1d = TrajectoryPoint1DRosMessage()
		thumb_point1d.position = thumb
		tdist.trajectory_points = [thumb_point1d] 

		thumb_roll1d = TrajectoryPoint1DRosMessage()
		thumb_roll1d.position = 1.5
		troll.trajectory_points = [thumb_roll1d] 

		pub.publish(msg)
		unid = unid + 1
		rate.sleep()

if __name__ == '__main__':
    try:
        glovenode()
    except rospy.ROSInterruptException:
        pass