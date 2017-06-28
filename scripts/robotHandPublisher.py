#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from numl_val_msgs.msg import HandPoseTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from sensor_msgs.msg import JointState



def glove_callback(data):
	global last_msg
	last_msg = data

def glove_to_robot_node():
	unid = 1
	rospy.init_node('glove_to_robot', anonymous=True)
	rate = rospy.Rate(100)
	pub = rospy.Publisher('/arm_control', HandPoseTrajectoryRosMessage, queue_size=1)
	sub = rospy.Subscriber("/glove_status", JointState, glove_callback)
	null_point1d = TrajectoryPoint1DRosMessage()
	null_point1d.position = 0.5
	null_onedof_1 = OneDoFJointTrajectoryRosMessage()
	null_onedof_2 = OneDoFJointTrajectoryRosMessage()
	null_onedof_3 = OneDoFJointTrajectoryRosMessage()

	right_msg = HandPoseTrajectoryRosMessage()
	right_msg.robot_side = 1 ##RIGHT
	right_msg.execution_mode = 0
	right_msg.desired_pose = 0
	right_msg.previous_message_id = unid - 1
	right_msg.homeAllForearmJoints = True
	right_troll = OneDoFJointTrajectoryRosMessage()
	right_tprox = OneDoFJointTrajectoryRosMessage()
	right_tdist = OneDoFJointTrajectoryRosMessage()
	right_index = OneDoFJointTrajectoryRosMessage()
	right_middle = OneDoFJointTrajectoryRosMessage()
	right_pinky = OneDoFJointTrajectoryRosMessage()
	right_fingers_list = [right_troll, right_tprox, right_tdist, right_index, right_middle, right_pinky]
	right_msg.hand_joint_trajectory_messages = left_fingers_list
    right_msg.forearm_joint_trajectory_messages = [null_onedof_1, null_onedof_2, null_onedof_3]

    left_msg = HandPoseTrajectoryRosMessage()
	left_msg.robot_side = 2 ##Double check that 2 is left
	left_msg.execution_mode = 0
	left_msg.desired_pose = 0
	left_msg.previous_message_id = unid - 1
	left_msg.homeAllForearmJoints = True
	left_troll = OneDoFJointTrajectoryRosMessage()
	left_tprox = OneDoFJointTrajectoryRosMessage()
	left_tdist = OneDoFJointTrajectoryRosMessage()
	left_index = OneDoFJointTrajectoryRosMessage()
	left_middle = OneDoFJointTrajectoryRosMessage()
	left_pinky = OneDoFJointTrajectoryRosMessage()
	left_fingers_list = [left_troll, left_tprox, left_tdist, left_index, left_middle, left_pinky]
	left_msg.hand_joint_trajectory_messages = left_fingers_list
    left_msg.forearm_joint_trajectory_messages = [null_onedof_1, null_onedof_2, null_onedof_3]

	for finger in left_fingers_list:
		finger.trajectory_points.append(null_point1d)
	for finger in right_fingers_list:
		finger.trajectory_points.append(null_point1d)

	while not rospy.is_shutdown():  ##There's a better way to do this than having pretty much the same code twice, shall be changed one day
		left_msg.unique_id = unid
		left_rin = last_msg.position[1]
		left_mid = last_msg.position[2]
		left_ind = last_msg.position[3]
		left_thumb = last_msg.position[4]

		left_rin = (left_rin * 2.7/3.0) + 0.3
		left_mid = (left_mid * 2.7/3.0) + 0.3
		left_ind = (left_ind * 2.7/3.0) + 0.3
		left_thumb = (left_thumb * 1.7/3.0) + 0.3

		left_pinky_point1d = TrajectoryPoint1DRosMessage()
		left_pinky_point1d.position = left_rin ##We use the value from the real life ring finger to control the robot's pinky
		left_pinky.trajectory_points = [left_pinky_point1d] 

		left_middle_point1d = TrajectoryPoint1DRosMessage()
		left_middle_point1d.position = left_mid
		left_middle.trajectory_points = [left_middle_point1d] 

		left_index_point1d = TrajectoryPoint1DRosMessage()
		left_index_point1d.position = left_ind
		left_index.trajectory_points = [left_index_point1d] 

		left_thumb_point1d = TrajectoryPoint1DRosMessage()
		left_thumb_point1d.position = left_thumb
		left_tdist.trajectory_points = [left_thumb_point1d] 

		left_thumb_roll1d = TrajectoryPoint1DRosMessage()
		left_thumb_roll1d.position = 1.5 ##thumb roll is fixed for now, this should probably be a no-move command instead, since we can't get thumb roll from the current glove
		left_troll.trajectory_points = [left_thumb_roll1d] 

		pub.publish(left_msg)
		unid = unid + 1

		right_msg.unique_id = unid
		right_rin = last_msg.position[6]
		right_mid = last_msg.position[7]
		right_ind = last_msg.position[8]
		right_thumb = last_msg.position[9]

		right_rin = (right_rin * 2.7/3.0) + 0.3
		right_mid = (right_mid * 2.7/3.0) + 0.3
		right_ind = (right_ind * 2.7/3.0) + 0.3
		right_thumb = (right_thumb * 1.7/3.0) + 0.3

		right_pinky_point1d = TrajectoryPoint1DRosMessage()
		right_pinky_point1d.position = right_rin ##We use the value from the real life ring finger to control the robot's pinky
		right_pinky.trajectory_points = [right_pinky_point1d] 

		right_middle_point1d = TrajectoryPoint1DRosMessage()
		right_middle_point1d.position = right_mid
		right_middle.trajectory_points = [right_middle_point1d] 

		right_index_point1d = TrajectoryPoint1DRosMessage()
		right_index_point1d.position = right_ind
		right_index.trajectory_points = [right_index_point1d] 

		right_thumb_point1d = TrajectoryPoint1DRosMessage()
		right_thumb_point1d.position = right_thumb
		right_tdist.trajectory_points = [right_thumb_point1d] 

		right_thumb_roll1d = TrajectoryPoint1DRosMessage()
		right_thumb_roll1d.position = 1.5 ##thumb roll is fixed for now, this should probably be a no-move command instead, since we can't get thumb roll from the current glove
		right_troll.trajectory_points = [right_thumb_roll1d] 

		pub.publish(right_msg)
		unid = unid + 1

		rate.sleep()

if __name__ == '__main__':
    try:
        glove_to_robot_node()
    except rospy.ROSInterruptException:
pass