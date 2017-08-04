#!/usr/bin/env python
import rospy
import roslib
import random
import math
import numpy as np
from numpy import ma
import tf
from filterpy.kalman import KalmanFilter
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment

# from plot_tool.srv import *

import matplotlib.pyplot as plt
from filterpy.stats import plot_covariance_ellipse

color_pallete = [ColorRGBA(0.8941176470588236, 0.10196078431372549, 0.10980392156862745, 0.3), ColorRGBA(0.21568627450980393, 0.49411764705882355, 0.7215686274509804, 0.3), ColorRGBA(0.30196078431372547, 0.6862745098039216, 0.2901960784313726, 0.3), ColorRGBA(0.596078431372549, 0.3058823529411765, 0.6392156862745098, 0.3), ColorRGBA(1.0, 0.4980392156862745, 0.0, 0.3), ColorRGBA(1.0, 1.0, 0.2, 0.3), ColorRGBA(0.6509803921568628, 0.33725490196078434, 0.1568627450980392, 0.3), ColorRGBA(0.9686274509803922, 0.5058823529411764, 0.7490196078431373, 0.3), ColorRGBA(0.6, 0.6, 0.6, 0.3)]


#######caution######
## current tracking will be in xz plane only, z will be managed once convex hull part is figured out
#####################


class tracked_object:
	''' Class to make individual tracked objects. More attributes can be added as detection algorithms evolve '''


	def __init__(self, x = 0.0, y = 0.0, z = 0.0, is_active = False, is_dead = False, youth = 0, strikes = 0, dt = 0.16): #dt = 0.1 bcoz we getting data at 10Hz
		'''
		x and yare coordinates of the the obstacles in the plane they are moving. is_active determines if a marker is to be published for this object.
		Youth variable is used to track the number of frames a new object appears, this is to filter out transient noise. Strikes calculate how long the object was not tracked.
		'''
		global color_pallete

		self.id = str(rospy.Time.now())
		self.x = x
		self.y = y
		self.z = z
		self.is_active = is_active
		self.is_dead = is_dead
		self.youth = youth
		self.strikes = strikes
		self.color = color_pallete[random.randint(0,8)]
		# kalman filter object. x = [x, x_dot, y, y_dot] and z = [x_measured, y_measured]
		self.kf = KalmanFilter (dim_x=4, dim_z=2)
		# state transition matrix
		self.kf.F = np.array([[1, dt, 0,  0],
													[0,  1, 0,  0],
													[0,  0, 1, dt],
													[0,  0, 0,  1]])
		# process noise matrix
		q = Q_discrete_white_noise(dim=2, dt=dt, var = 2) # orignal value 200
		self.kf.Q = block_diag(q, q)
		# measurement funtion
		self.kf.H = np.array([[1, 0, 0, 0],
													[0, 0, 1, 0]])
		# measurement noise matrix
		self.kf.R = np.array([[0.05, 0.],
													[0., 0.05]])   #orignal values .2
		self.kf.x = np.array([[self.x, 0, self.y, 0]]).T ######it is z, not y for the time being
		# self.kf.P = np.eye(4) * 100.
		self.kf.P = np.array([[1,  0, 0,  0], #######original value 2
													[0, 4, 0,  0],
													[0,  0, 1,  0],
													[0,  0, 0, 4]])
		# can be added later
		#self.frame

	def update_matrices(self, dt):
		self.kf.F = np.array([[1, dt, 0,  0],
													[0,  1, 0,  0],
													[0,  0, 1, dt],
													[0,  0, 0,  1]])
		q = Q_discrete_white_noise(dim=2, dt=dt, var=0.001)
		self.kf.Q = block_diag(q, q)

	def activate(self):
		self.is_active = True

	def deactivate(self):
		self.is_active = False

	def dead(self):
		self.is_dead = True

	def __str__(self):
		return "x=%s, y=%s, z=%s, youth=%s, strikes=%s, is_active=%s" % (self.x, self.y, self.z, self.youth, self.strikes, self.is_active)

class Mot:
	'''Multi object tracker class'''

	marker = Marker()
	marker.header.frame_id = "/base_link"

	marker.ns = "mot_ojects"
	# marker.id = 1
	marker.type = Marker.CUBE_LIST
	marker.action = Marker.ADD
	marker.pose.position.x = 0
	marker.pose.position.y = 0
	marker.pose.position.z = 0
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 1
	marker.scale.y = 1
	marker.scale.z = 1
	# marker.lifetime = rospy.Duration(2)

	def __init__(self):
		# self.tracker_pub = rospy.Publisher("/tracked_markers", MarkerArray, queue_size = 10)
		self.tracker_list_pub = rospy.Publisher("/tracked_marker_list", Marker, queue_size = 10)
		# self.velocity_pub = rospy.Publisher("/velocity_viz", Point, queue_size = 10)
		self.centoid_sub = rospy.Subscriber("/object_centroids", PoseArray, self.centroid_callback, queue_size = 10)
		self.tracked_objects = []
		self.distance_threshold = 1
		self.youth_threshold = 3
		self.strikes_threshold = 3
		self.newDataAvailable = False




		self.listener = tf.TransformListener()
		self.transformer = tf.TransformerROS()
		self.trans_matrix = None

	def centroid_callback(self, ros_data):
		self.newDataAvailable = True
		self.latest_header = ros_data.header
		self.latest_objects = ros_data.poses
		self.marker.header.frame_id = ros_data.header.frame_id



		try:
			now = rospy.Time.now()
 			self.listener.waitForTransform("/latest_centroids", "/prev_centroids", now, rospy.Duration(4.0))
			(trans,rot) = self.listener.lookupTransform('/latest_centroids', '/prev_centroids', now)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "waiting for tf"

		self.trans_matrix = self.transformer.fromTranslationRotation(trans, rot)
		# print self.trans_matrix




		#if the message was empty increase strikes for everyone, and if some object has strikes greater than the threshold, deactivate the object
		if(len(self.latest_objects)==0):
			for obj in self.tracked_objects:
				obj.strikes += 1
				if obj.strikes > self.strikes_threshold:
					obj.dead()
			# removing deactivated objects
			self.tracked_objects = [obj for obj in self.tracked_objects if not (obj.is_dead)]
			# print str(self.tracked_objects)
		else:
			# print "doing assignment"
			self.do_assignment()
		# self.newDataAvailable = False

	def do_assignment(self):

		# if there are currently no tracked objects, start tracking all the objects in the recieved message
		if not self.tracked_objects:
			for i in self.latest_objects:
				self.tracked_objects.append(tracked_object(i.position.x, i.position.y, i.position.z))
			return

		# print "latest centroids"
		# for i in self.latest_objects:
		# 	print i.position.x, i.position.y, i.position.z


		# separate out the centroids from tracked objects and and the new objects ros message lists and convert them to numpy arrays
		tracked_centroids_coord = np.matrix([(i.x, i.y, i.z, 1) for i in self.tracked_objects])
		#
		tracked_centroids_coord_odom = self.trans_matrix * tracked_centroids_coord.T




		# print "odometry"
		# print np.shape(tracked_centroids_coord)
		# print tracked_centroids_coord


		for  index in range(len(self.tracked_objects)):
			# print "index: %d index" % index
			# print tracked_centroids_coord_odom[0,index],tracked_centroids_coord_odom[1,index],tracked_centroids_coord_odom[2,index]
			self.tracked_objects[index].x = tracked_centroids_coord_odom[0,index]
			self.tracked_objects[index].y = tracked_centroids_coord_odom[1,index]
			self.tracked_objects[index].z = tracked_centroids_coord_odom[2,index]




		# separate out the centroids from tracked objects and and the new objects ros message lists and convert them to numpy arrays
		tracked_centroids = np.asarray([(i.x, i.y, i.z) for i in self.tracked_objects])




		latest_centroids = np.asarray([(i.position.x, i.position.y, i.position.z) for i in self.latest_objects])

		#rospy.loginfo(str(len(tracked_centroids)) +", "+ str(len(latest_centroids)) )
		# calculating the costmatrix based on euclidean distance and applying munkres assignment to get row and column indices
		# row_ind, col_ind <--> tracked_centroids, latest_centroids
		cost_matrix = distance.cdist(tracked_centroids,latest_centroids,'euclidean')
		row_ind, col_ind = linear_sum_assignment(cost_matrix)

		#iterate over pairs to see if they are not too far
		pairs = np.array([]).reshape(0,2)
		for index in range(len(row_ind)):
			if cost_matrix[row_ind[index],col_ind[index]] <  self.distance_threshold:
				pairs = np.vstack([pairs,[row_ind[index],col_ind[index]]])

		# for all the the old objects that have matching new centroids, update the centroids. Increase the youth value. If youth value is over a threshold, activate the trackers
		pairs = pairs.astype(int)
		if len(pairs) != 0:

			for index in range(len(pairs)):
				# self.tracked_objects[pairs[index,0]].x = latest_centroids[pairs[index,1]][0]
				# self.tracked_objects[pairs[index,0]].y = latest_centroids[pairs[index,1]][1]
				# self.tracked_objects[pairs[index,0]].z = latest_centroids[pairs[index,1]][2]
				z_kf = np.array([[latest_centroids[pairs[index,1]][0]],[latest_centroids[pairs[index,1]][1]]])
				self.tracked_objects[pairs[index,0]].kf.update(z_kf)
				# updating our own instance variables with updated values from the kf object
				self.tracked_objects[pairs[index,0]].x = float(self.tracked_objects[pairs[index,0]].kf.x[0])
				self.tracked_objects[pairs[index,0]].y = float(self.tracked_objects[pairs[index,0]].kf.x[2])

				#if the match is found then make the strikes zero
				self.tracked_objects[pairs[index,0]].strikes = 0

				if self.tracked_objects[pairs[index,0]].youth < self.youth_threshold:
					self.tracked_objects[pairs[index,0]].youth += 1
				else:
					self.tracked_objects[pairs[index,0]].activate()

		# increasing strikes for objects that were being tracked, but not matched in this iteration. Then remove objects with strikes greater than the threshold
		mask_currently_tracked = np.in1d(np.arange(len(self.tracked_objects)), pairs[:,0])
		increase_strikes = np.where(~mask_currently_tracked)[0]
		if len(increase_strikes) != 0:
			for x in np.nditer(increase_strikes):
				self.tracked_objects[x].strikes += 1
				if self.tracked_objects[x].strikes > self.strikes_threshold:
					self.tracked_objects[x].dead()

			# removing deactivated objects
			self.tracked_objects = [obj for obj in self.tracked_objects if not (obj.is_dead)]


		# append newly detected objects to already tracked objects
		mask_new_detections = np.in1d(np.arange(len(latest_centroids)), pairs[:,1])
		new_trackers_to_start = np.where(~mask_new_detections)[0]
		if len(new_trackers_to_start) !=0:
			for x in np.nditer(new_trackers_to_start):
				self.tracked_objects.append(tracked_object(latest_centroids[x,0],latest_centroids[x,1], latest_centroids[x,2]))

		# print "tracked centroids"
		# for i in self.tracked_objects:
		# 	x_vel = i.kf.x[1][0]
		# 	y_vel = i.kf.x[3][0]
		# 	norm_vel = math.sqrt(x_vel**2 + y_vel**2)
		# 	print i.x, i.y, i.youth, i.strikes, norm_vel


def main():
	rospy.init_node('multi_object_tracker', anonymous=False)
	mot = Mot()
	rospy.loginfo('Starting multi-object tracking node...')
	rate = rospy.Rate(20) # 20hz
	while not mot.newDataAvailable and not rospy.is_shutdown():
		rospy.loginfo("Waiting for first message...")
		#rospy.loginfo(str(mot.newDataAvailable) +" "+ str(rospy.is_shutdown())
		print mot.newDataAvailable, rospy.is_shutdown()
		rospy.sleep(1.0)

	print "here1"
	# loop_count = 0
	# plt.ion()
	# fig=plt.figure()
	while not rospy.is_shutdown():
		Mot.marker.action = 2 #DELETEALL
		Mot.marker.colors = []
		Mot.marker.points = []
		mot.tracker_list_pub.publish(Mot.marker)

		for index, obj in enumerate(mot.tracked_objects):
			# predict new means
			obj.kf.predict()
			obj.x = float(obj.kf.x[0])
			obj.y = float(obj.kf.x[2])
			# populating and publishing markers
			#print obj.is_active

			if obj.is_active:
				# print obj
				Mot.marker.action = Marker.ADD
				Mot.marker.header.stamp = rospy.Time()#mot.latest_header.stamp
				Mot.marker.colors.append(obj.color)
				Mot.marker.points.append(Point(obj.x, obj.y, obj.z))


				#rospy.loginfo("Centroid "+ str(index) + " :" + str(obj.x) +", "+ str(obj.y) + ", "+ str(obj.z))
				#################
				#rospy.loginfo(str(obj.kf.P))
				# str_to_append = obj.id + ", " + str(obj.kf.P[0][0]) + "," + str(obj.kf.P[2][0]) + "," + str(obj.kf.P[0][2]) + "," + str(obj.kf.P[2][2]) + "," + str(obj.kf.x[0][0]) + "," + str(obj.kf.x[2][0]) + "\n"
				# print str_to_append
				# with open('/home/sig/plot_data.txt', 'a') as file:
				# 	file.write(str_to_append)



				# if index == 0:
				# 	x_vel = obj.kf.x[1][0]
				# 	y_vel = obj.kf.x[3][0]
				# 	norm_vel = math.sqrt(x_vel**2 + y_vel**2)
				# 	mot.velocity_pub.publish(norm_vel, 0. , 0.)





				# if loop_count % 5 == 0:
				# 	cov = np.array([[obj.kf.P[0][0],obj.kf.P[2][0]],
				# 									[obj.kf.P[0][2],obj.kf.P[2][2]]])
				# 	mean = (obj.kf.x[0][0], obj.kf.x[2][0])
					# cov = np.array([[obj.kf.P[1][1],obj.kf.P[3][1]],
													# [obj.kf.P[1][3],obj.kf.P[3][3]]])
					# mean = (obj.kf.x[1][0], obj.kf.x[3][0])
					# plot_covariance_ellipse(mean, cov=cov, fc='r', std=1, alpha=0.2)
					# plt.pause(0.001)
				#################
			## rospy.wait_for_service('/plot_tool/draw_pose')
			## try:
			## 	draw_pose = rospy.ServiceProxy('/plot_tool/draw_pose', PlotPose)
			## 	resp1 = draw_pose(Mot.marker.pose, int(index) , True, ord('o'), 1)
			## except rospy.ServiceException, e:
			## 	print "Service call failed: %s"%e
		mot.tracker_list_pub.publish(Mot.marker)
		# loop_count += 1
		rate.sleep()

	rospy.loginfo("Shutting down Multi Object tracker node")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
