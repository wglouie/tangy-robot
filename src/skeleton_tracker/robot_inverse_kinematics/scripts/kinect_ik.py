#!/usr/bin/env python
import numpy as np
import sys
import csv
from pprint import pprint
import os
import rospkg
import rospy
import tf
import time
from std_msgs.msg import String
from skeletonmsgs_nu.msg import *
from drrobot_h20_arm_player.msg import *
from trajectory_executor.msg import *
from trajectory_executor.srv import *
from robot_inverse_kinematics.srv import *
from collision_avoider.srv import *

FREQ_DIV = 30   #frequency divider for checking "key" skeleton
ANG_MULT = 20
DIST_MULT = 1
MAX_FRAMES = 1
ANGLE_THRESHOLD = 1; #0.5

class KinectIK(object):
	def __init__(self):
		self.node = rospy.init_node('robot_inverse_kinematics')

		self.tflistener = tf.TransformListener()

		self.skeletons_sub = rospy.Subscriber('/skeletons', Skeletons, self.skeletons_cb)
		self.left_arm_traj_pub = rospy.Publisher('trajectory_executor/left_arm_traj',
											arm_trajectory, queue_size=1)
		self.right_arm_traj_pub = rospy.Publisher('trajectory_executor/right_arm_traj',
												  arm_trajectory, queue_size=1)

		self.record_traj_sub = rospy.Subscriber('/robot_ik/record', String, self.record_traj_cb)
		self.save_traj_sub = rospy.Subscriber('/robot_ik/save_to_file', String, self.save_traj_cb)
		self.exec_traj_file_srv = rospy.Service('/robot_ik/execute_file', execute_gesture_file, self.exec_traj_file_cb)

		self.left_arm_angle_data = []
		self.right_arm_angle_data = []
		self.head_angle_data = []

		self.last_published_right = [0, 0, 0, 0]
		self.last_published_left = [0, 0, 0, 0]

		self.left_arm_record = []
		self.right_arm_record = []

		self.is_record_started = 0
		self.key_index = 0
		self.key_id = 1
		self.count = 0
		self.file_count = 0

	def record_traj_cb(self,data):
		msg = data.data
		if msg == 'start':
			self.is_record_started = 1
			self.left_arm_record = []
			self.right_arm_record = []
			print "Start recording trajectories"
		elif msg == 'stop':
			self.is_record_started = 0
			print "Stop recording trajectories"

	def save_traj_cb(self,data):
		file_name = data.data
                self.ang_save(file_name + '_left_arm.gesture', self.left_arm_record)
                self.ang_save(file_name + '_right_arm.gesture', self.right_arm_record)
		print "Save trajectory to file path: [" + file_name + "]"

	def exec_traj_file_cb(self,req):
		file_name = req.file_path
		left_arm_trajectory = []
		right_arm_trajectory = []
		try:
                        with open(file_name + '_left_arm.gesture', 'r') as f:
				records = csv.reader(f, delimiter="\t")
				for row in records:
					left_arm_trajectory.append([float(i) for i in row])

                        with open(file_name + '_right_arm.gesture', 'r') as f:
				records = csv.reader(f, delimiter="\t")
				for row in records:
					right_arm_trajectory.append([float(i) for i in row])

			print "Execute trajectory from file path: [" + file_name + "]"
			self.publish_both_trajectories(left_arm_trajectory, right_arm_trajectory)
			print "FAIL HERE"
			return  execute_gesture_fileResponse(True)
		except IOError:
			rospy.logerr("ERROR: could not find path \'" + file_name + '_left_arm' + "\'. Cannot execute trajectory file")
			return  execute_gesture_fileResponse(False)

	def skeletons_cb(self,data):
		if self.is_record_started == 1:
			if len(data.skeletons) == 0:
				return
			if self.count % FREQ_DIV == 0:
				self.get_key_user(data.skeletons)
			self.count += 1
			if self.key_index < len(data.skeletons) and \
							data.skeletons[self.key_index].userid == self.key_id:
				skel = data.skeletons[self.key_index]
			else:
				for i, skel in enumerate(data.skeletons):
					if skel.userid == self.key_id:
						found = True
						break
					found = False
				if not found:
					rospy.logwarn("Could not find a skeleton userid that matches the key user")
					return

			user = skel.userid

			if (self.tflistener.frameExists('/torso_' + str(user)) and
					self.tflistener.frameExists('/left_hand_' + str(user))):
				try:
					left_arm_pos = {'hand': None, 'elbow': None, 'shoulder': None}
					right_arm_pos = {'hand': None, 'elbow': None, 'shoulder': None}


					(left_arm_pos['hand'],dump1) = self.tflistener.lookupTransform('/skeleton_kinect2',
																	 '/left_hand_' + str(user), rospy.Time(0))
					(left_arm_pos['elbow'], dump2) = self.tflistener.lookupTransform('/skeleton_kinect2',
																	 '/left_elbow_' + str(user), rospy.Time(0))
					(left_arm_pos['shoulder'], dump3) = self.tflistener.lookupTransform('/skeleton_kinect2',
																	 '/left_shoulder_' + str(user), rospy.Time(0))
					(right_arm_pos['hand'], dump4) = self.tflistener.lookupTransform('/skeleton_kinect2',
																	 '/right_hand_' + str(user), rospy.Time(0))
					(right_arm_pos['elbow'], dump5) = self.tflistener.lookupTransform('/skeleton_kinect2',
																	 '/right_elbow_' + str(user), rospy.Time(0))
					(right_arm_pos['shoulder'], dump6) = self.tflistener.lookupTransform('/skeleton_kinect2',
																	 '/right_shoulder_' + str(user), rospy.Time(0))


					if self.file_count % 50 == 0:
						print "Left_Hand: "

					left_arm_vec = self.calc_vectors(left_arm_pos)
					right_arm_vec = self.calc_vectors(right_arm_pos)


					left_arm_angles = self.calc_left_angles(left_arm_vec)
					right_arm_angles = self.calc_right_angles(right_arm_vec)


					left_arm_angles = self.calc_left_angles(left_arm_vec)
					right_arm_angles = self.calc_right_angles(right_arm_vec)

					self.ang_save("left_arm_angles",left_arm_angles)

					self.ang_save("right_arm_angles",right_arm_angles)


					# left
					self.left_arm_angle_data.append(left_arm_angles)


					# Right
					self.right_arm_angle_data.append(right_arm_angles)

					#Check collision
					is_colliding = self.check_collision_client(left_arm_angles, right_arm_angles)
					if is_colliding[0]:
						print "Left Collision!  Left Collision! Left Collision!"
					else:
						self.left_arm_record.append(left_arm_angles)
						self.publish_left_trajectory(self.left_arm_angle_data, self.left_arm_traj_pub)
					if is_colliding[1]:
						print "Right Collision!  Right Collision! Right Collision!"
					else:
						self.right_arm_record.append(right_arm_angles)
						self.publish_right_trajectory(self.right_arm_angle_data, self.right_arm_traj_pub)

					self.left_arm_angle_data = []
					self.right_arm_angle_data = []

					self.file_count += 1


				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					pass

			return

	def joint_pos_save(self,file_name, joint_positions,user):
		try:
			with open(file_name, "a") as f:
				f.write("\n\nhand_"+ str(user) + "\t")
				for value in joint_positions['hand']:
					f.write("%s, " % "{:10.4f}".format(value))
				f.write("\n\nelbow_"+ str(user) + "\t")
				for value in joint_positions['elbow']:
					f.write("%s, " % "{:10.4f}".format(value))
				f.write("\n\nshoulder_"+ str(user) + "\t")
				for value in joint_positions['shoulder']:
					f.write("%s, " % "{:10.4f}".format(value))
				f.write("\n")
		except IOError:
			rospy.logerr("ERROR: could not find path \'" + file_name + "\'. Can't save positions.")

	def arm_vecs_save(self,file_name,arm_vecs):
		try:
			with open(file_name, "a") as f:
				f.write("elbow_shoulder ")
				np.savetxt(f, arm_vecs[0], fmt='%-7.2f', newline=" ")
				f.write("\nhand_elbow ")
				np.savetxt(f, arm_vecs[1], fmt='%-7.2f', newline=" ")
				f.write("\nhand_shoulder ")
				np.savetxt(f, arm_vecs[2], fmt='%-7.2f', newline=" ")
				f.write("\n\n")
		except IOError:
			rospy.logerr("ERROR: could not find path \'" + file_name + "\'. Can't save vectors.")
	def head_vecs_save(self,file_name,head_vecs):
		try:
			with open(file_name, "a") as f:
				f.write("head_neck ")
				np.savetxt(f, head_vecs[0], fmt='%-7.2f', newline=" ")
				f.write("\n\n")
		except IOError:
			rospy.logerr("ERROR: could not find path \'" + file_name + "\'. Can't save vectors.")
	def ang_save(self,file_name,angles):
		try:
			with open(file_name, "a") as f:
				np.savetxt(f, angles, fmt='%-7.2f', newline="\n", delimiter="\t")
		except IOError:
			rospy.logerr("ERROR: could not find path \'" + file_name + "\'. Can't save angles.")

	def get_key_user(self, skels):
		data = []
		for i, s in enumerate(skels):
			v2 = np.array([s.head.transform.translation.x,
						   s.head.transform.translation.z])
			ang = np.arccos(abs(v2[1]) / np.linalg.norm(v2))
			dist = v2[1]
			cost = ANG_MULT * ang + DIST_MULT * dist
			data.append([i, s.userid, cost])
		val, idx = min((val[2], idx) for (idx, val) in enumerate(data))
		self.key_index = data[idx][0]
		self.key_id = data[idx][1]
		return

	def calc_vectors(self, arm_pos):
		elbow_shoulder_vec = np.subtract(arm_pos['elbow'],arm_pos['shoulder'])
		hand_elbow_vec = np.subtract(arm_pos['hand'],arm_pos['elbow'])
		hand_shoulder_vec = np.subtract(arm_pos['hand'],arm_pos['shoulder'])
		return [elbow_shoulder_vec,hand_elbow_vec,hand_shoulder_vec]


	def calc_left_angles(self,arm_vec):

		temp0 = self.mapangle(np.arcsin(arm_vec[0][1] / np.sqrt(np.square(arm_vec[0][0]) + np.square(arm_vec[0][1]) + np.square(arm_vec[0][2]))), -1.5708, 1.57008, 0,3.14159)
		temp1 = np.arcsin(arm_vec[0][0] / np.sqrt(np.square(arm_vec[0][0]) + np.square(arm_vec[0][1]) + np.square(arm_vec[0][2])))
		temp2 = np.arcsin(arm_vec[1][0] / np.sqrt(np.square(arm_vec[1][0]) + np.square(arm_vec[1][1]) + np.square(arm_vec[1][2])))
		l1 = np.array(arm_vec[0])
		l2 = np.array(arm_vec[1])
		l3 = np.array(arm_vec[2])
		temp3 = np.arccos( (np.square(np.linalg.norm(l3)) - np.square(np.linalg.norm(l1)) - np.square(np.linalg.norm(l2))) / (2 * np.linalg.norm(l1) * np.linalg.norm(l2) ) )
		return [temp0, temp1, temp2, -1*temp3]
	

	def calc_right_angles(self,arm_vec):

		temp0 = self.mapangle(np.arcsin(arm_vec[0][1] / np.sqrt(np.square(arm_vec[0][0]) + np.square(arm_vec[0][1]) + np.square(arm_vec[0][2]))), -1.5708, 1.57008, 0,3.14159)
		temp1 = np.arcsin(arm_vec[0][0] / np.sqrt(np.square(arm_vec[0][0]) + np.square(arm_vec[0][1]) + np.square(arm_vec[0][2])))
		temp2 = np.arcsin(arm_vec[1][0] / np.sqrt(np.square(arm_vec[1][0]) + np.square(arm_vec[1][1]) + np.square(arm_vec[1][2])))
		l1 = np.array(arm_vec[0])
		l2 = np.array(arm_vec[1])
		l3 = np.array(arm_vec[2])
		temp3 = np.arccos( (np.square(np.linalg.norm(l3)) - np.square(np.linalg.norm(l1)) - np.square(np.linalg.norm(l2))) / (2 * np.linalg.norm(l1) * np.linalg.norm(l2) ) )
		return [temp0, -1 * temp1, -1 * temp2, -1*temp3]


	def mapangle(self,angle_value,range1_min,range1_max,range2_min,range2_max):
		return range2_min+(angle_value-range1_min)*(range2_max-range2_min)/(range1_max-range1_min)



	def publish_right_trajectory(self,trajectory,publisher):
		goal = arm_trajectory()

		for i in range(len(trajectory)):
			if self.is_bad_angle(trajectory[i],self.last_published_right) == False:
				point = arm_cmd()
				point.velocity = 60
				point.joint_commands[0].joint_angle = 0
				point.joint_commands[1].joint_angle = 0
				point.joint_commands[2].joint_angle = trajectory[i][3] 
				point.joint_commands[3].joint_angle = trajectory[i][2] 
				point.joint_commands[4].joint_angle = trajectory[i][1] 
				point.joint_commands[5].joint_angle = trajectory[i][0] 
				goal.trajectory.append(point)
				self.last_published_right = trajectory[i]

		goal.header.stamp = rospy.get_rostime()
		publisher.publish(goal)
		print "Publish Right Arm Trajectory"

	def publish_left_trajectory(self,trajectory,publisher):
		goal = arm_trajectory()
	
		for i in range(len(trajectory)):
			if self.is_bad_angle(trajectory[i],self.last_published_left) == False:
				point = arm_cmd()
				point.velocity = 60
				point.joint_commands[0].joint_angle = 0
				point.joint_commands[1].joint_angle = 0
				point.joint_commands[2].joint_angle = trajectory[i][3]
				point.joint_commands[3].joint_angle = trajectory[i][2]  
				point.joint_commands[4].joint_angle = trajectory[i][1]
				point.joint_commands[5].joint_angle = trajectory[i][0] 
				goal.trajectory.append(point)
				self.last_published_left = trajectory[i]
		goal.header.stamp = rospy.get_rostime()
		publisher.publish(goal)

		print "Publish Left Arm Trajectory"

	def publish_both_trajectories(self, left_trajectory, right_trajectory):
		goal = both_arm_trajectory()

		for i in range(len(left_trajectory)):
			if self.is_bad_angle(left_trajectory[i],self.last_published_left) == False:
				point = arm_cmd()
				point.velocity = 60
				point.joint_commands[0].joint_angle = 0
				point.joint_commands[1].joint_angle = 0
				point.joint_commands[2].joint_angle = left_trajectory[i][3] 
				point.joint_commands[3].joint_angle = left_trajectory[i][2] 
				point.joint_commands[4].joint_angle = left_trajectory[i][1] 
				point.joint_commands[5].joint_angle = left_trajectory[i][0]
				goal.left_trajectory.append(point)
				self.last_published_left = left_trajectory[i]

		for i in range(len(right_trajectory)):
			if self.is_bad_angle(right_trajectory[i],self.last_published_right) == False:
			
				point = arm_cmd()
				point.velocity = 60
				point.joint_commands[0].joint_angle = 0
				point.joint_commands[1].joint_angle = 0
				point.joint_commands[2].joint_angle = right_trajectory[i][3]
				point.joint_commands[3].joint_angle = right_trajectory[i][2]
				point.joint_commands[4].joint_angle = right_trajectory[i][1]
				point.joint_commands[5].joint_angle = right_trajectory[i][0]
				goal.right_trajectory.append(point)
				self.last_published_right = right_trajectory[i]
		goal.header.stamp = rospy.get_rostime()

		print "Publish Both Arm Trajectories"

		try:
			req = rospy.ServiceProxy('trajectory_executor/both_arm_traj',
									 execute_both_arms)
			res = req(goal)
			return res.success
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

	def is_bad_angle(self, trajectory_point, last_angle):
		for i in range(len(trajectory_point)):
			angle_difference = trajectory_point[i] - last_angle[i] 
			if abs(angle_difference) > ANGLE_THRESHOLD:
				return True

		return False




	def check_collision_client(self,left_arm_angs, right_arm_angs):
		try:
			check_collision_req = rospy.ServiceProxy('collision_avoider/check_collision',
													  check_collision)
			res = check_collision_req(left_arm_angs,right_arm_angs)
			return [res.is_there_collision_left, res.is_there_collision_right]
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
			return

def main():
	IKSolver = KinectIK()
	rospy.spin()

if __name__ == "__main__":
	main()
