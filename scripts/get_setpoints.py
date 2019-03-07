#!/usr/bin/env python
import os
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from rosflight_msgs.msg import RCRaw
from reef_msgs.msg import DesiredState


#this class take a list of setpoints and some velocity parameters as arguments. Therefore, it publishs linear and angular velocities to apply to a robot in order to go through a dubins path
class dubins_velocity_publisher():
	def __init__(self,loop_rate):

		#initalisation of the flag variables
		self.loop_rate = loop_rate
		self.initialisation = True
		self.end = False
		self.infinite_finish_loop = False
		self.not_activated = True #not publish any more path points

		#declaration of the differents publishers
		# the publishers below are used for debugging the application
		self.gains_pub = rospy.Publisher("gains_publisher", Twist, queue_size=10)
		self.path_pub = rospy.Publisher("path_publisher", PoseStamped, queue_size=10)
		# the publishers below are useful to send some commands to the turtlebot
		self.velocity_stamped_pub = rospy.Publisher("velocity_stamped_publisher", TwistStamped, queue_size=10)
		#the publishers below are useful to send some commands to the quad
		self.quad_desired_state_pub = rospy.Publisher("desired_state", DesiredState, queue_size=10)

		#two mocap subscribers are availables in order to get the informations from the rosvrpn (message types = PoseStamped and  TransformStamped)
		rospy.Subscriber("pose_stamped", PoseStamped, self.mocap_pose_callback)
		rospy.Subscriber("pose", TransformStamped, self.mocap_transform_callback)
		rospy.Subscriber("rc_raw",RCRaw,self.rc_raw_callback)

		self.initial_yaw = rospy.get_param("~initial_yaw")
		#the differents setpoints are retrieved in this variable
		self.waypoint_list = rospy.get_param("~waypoint_list")

		#if the user wants to reproduce the dubins path multiple times, he can set a number of cycle
		num_cycles = rospy.get_param("~number_of_cycles")
		if type(num_cycles) == int and num_cycles > 0:
			self.waypoint_list *= rospy.get_param("~number_of_cycles")


		#these parameters set the radius that the robot has to perform around each setpoint and the delimitation of the setpoint in space
		self.setpoint_radius_tolerance = rospy.get_param("~setpoint_radius_tolerance")

		#theses parameters set the VMax, Vmin (minmum and maximum linear velocity) and PhiMax (maximum angular velocity) of the robot
		self.VMax = rospy.get_param("~VMax")
		self.VMin = rospy.get_param("~VMin")
		self.PhiMax = rospy.get_param("~PhiMax")

		# theses parameters set the gains (in x,y,theta)of the robot
		self.Kx = rospy.get_param("~Kx")
		self.Ky = rospy.get_param("~Ky")
		self.Original_Kx = rospy.get_param("~Kx")
		self.Original_Ky = rospy.get_param("~Ky")
		self.Ktheta = rospy.get_param("~Ktheta")

		#load the command orientation of the robot to send the right angular velocity
		if(rospy.get_param("~robot_command_orientation") == "NED"):
			self.robot_command_orientation = 1
		elif(rospy.get_param("~robot_command_orientation") == "NWU"):
			self.robot_command_orientation = -1

		#this parameter is used in order to slow down or speed up the robot when he is in a circular trajectory. It will add or remove some point of the current circular path.
		self.circular_velocity_ratio = rospy.get_param("~circular_velocity_ratio")
		# this parameter defines the radius's size of the robot on the ground
		self.robot_radius = rospy.get_param("~robot_radius")

		#this parameter is used to set altitude of the robot when the dubins path is used with quads
		self.altitude = rospy.get_param("~altitude")


		# initialisation of the path that the robot will follow
		self.path = {}
		# this parameter sets the distance between each point of the path.
		self.distance_between_point = rospy.get_param("~distance_between_point")
		#this parameter sets the rate of the path s evolution
		self.path_rate = rospy.get_param("~path_rate")
		self.original_path_rate = rospy.get_param("~path_rate")

		#this parameter defines the area of the tangent point on the setpoints relatively to the setpoint radius
		self.tangent_point_radius_tolerance = rospy.get_param("~proportional_tangent_point_radius_tolerance")
		self.tangent_point_radius_tolerance = self.tangent_point_radius_tolerance * self.setpoint_radius_tolerance

		#initialisation of the published messages
		self.path_pose = PoseStamped()
		self.gains = Twist()
		self.setvelocity_msg = TwistStamped()
		self.setvelocity_msg.header.stamp = rospy.Time.now()
		self.setvelocity_msg.twist.linear.x = 0
		self.setvelocity_msg.twist.angular.z = 0
		self.quad_desired_state_msg = DesiredState()
		self.quad_desired_state_msg.velocity_valid = True

		#initialisation of the first state: this is the first set_point, the robot is not in a transition state (from straight to circular or from circular to straight)
		self.waypoint_index = 0
		self.setpoint_msg, self.setpoint_direction = self.pose_msg_from_dict(self.waypoint_list[self.waypoint_index])
		self.initial_straight_velocity = self.VMin

		# initialisation of the end point of the setpoint
		if len(self.waypoint_list) > 1 :
			self.next_setpoint_msg,self.next_setpoint_direction = self.pose_msg_from_dict(self.waypoint_list[self.waypoint_index + 1])
			if self.setpoint_direction == 0:
				self.current_end_point = self.compute_tangent_setpoint_entry_point(1,
																				   self.next_setpoint_msg.pose.position.x,
																				   self.next_setpoint_msg.pose.position.y,
																				   self.setpoint_msg.pose.position.x,
																				   self.setpoint_msg.pose.position.y,
																				   self.setpoint_radius_tolerance)
			elif self.setpoint_direction == 1:
				self.current_end_point = self.compute_tangent_setpoint_entry_point(0,
																				   self.next_setpoint_msg.pose.position.x,
																				   self.next_setpoint_msg.pose.position.y,
																				   self.setpoint_msg.pose.position.x,
																				   self.setpoint_msg.pose.position.y,
																				   self.setpoint_radius_tolerance)



	#this function publishs the velocities messages and controles the path evolution and the gains
	def spin(self):
		#don t spin before the initialisation is finished
		if self.initialisation or self.not_activated:
			self.count = 0
			self.pose_valid = True
			self.velocity_valid = False
			self.quad_desired_state_msg.pose.x = 0
			self.quad_desired_state_msg.pose.y = 0
			self.quad_desired_state_msg.pose.yaw = self.initial_yaw
		else:
			#check if the node rate is not superior of the the path rate set by the user
			if self.loop_rate < self.path_rate:
				print "error the the path rate must be inferior to the node rate"
			else:
				#computation of the distance between the current poisition and the next point on the path
				dist_to_next_point = math.sqrt(
					math.pow((self.path["y"][self.path_index] - self.current_position[1]), 2) + (math.pow((self.path["x"][self.path_index] - self.current_position[0]), 2)))
				#the points of the path which are under the robot, are eliminated
				while((dist_to_next_point < self.robot_radius )and (self.path_index < self.number_path_point-1)):
					self.path_index = self.path_index + 1
					dist_to_next_point = math.sqrt(
					math.pow((self.path["y"][self.path_index] - self.current_position[1]), 2) + (math.pow((self.path["x"][self.path_index] - self.current_position[0]), 2)))


				#as we want a smooth acceleration, we play on gains. So for the 1/3 of the current path, the gains will evolve from (self.initial_straight_velocity / self.VMax) until there original value.
				#we make also sure that the VMax won't be exceeded by the robot
				if ( float(self.path_index)/float(self.number_path_point) < 0.33 and self.Kx <= self.Original_Kx and self.Ky <= self.Original_Ky) and (not(self.infinite_finish_loop)):
					self.Kx = (self.initial_straight_velocity / self.VMax)*self.Original_Kx + 3.*(float(self.path_index)/float(self.number_path_point))*(1.-(self.initial_straight_velocity / self.VMax))*self.Original_Kx
					self.Ky = (self.initial_straight_velocity / self.VMax)*self.Original_Ky + 3.*(float(self.path_index)/float(self.number_path_point))*(1.-(self.initial_straight_velocity / self.VMax))*self.Original_Ky

				else:
					self.Kx = self.Original_Kx
					self.Ky = self.Original_Ky

				#we compute the rate factor in order to make the path evolve at the rate specified by the user
				rate_factor = int(self.loop_rate / self.path_rate)
				if (self.path_index < self.number_path_point-1) and (self.count % rate_factor == 0):
					self.path_index = self.path_index + 1



				#all the velocities messages published are set below
				self.setvelocity_msg.header.stamp = rospy.Time.now()

				self.pose_valid = False
				self.velocity_valid = True
				self.quad_desired_state_msg.velocity.x = self.setvelocity_msg.twist.linear.x
				self.quad_desired_state_msg.velocity.yaw = self.setvelocity_msg.twist.angular.z

				self.velocity_stamped_pub.publish(self.setvelocity_msg)

				# For debugging, we publish the current gains and the current path position
				self.gains.linear.x = self.Kx
				self.gains.linear.y = self.Ky
				self.gains_pub.publish(self.gains)
				self.path_pose.pose.position.x = self.path["x"][self.path_index]
				self.path_pose.pose.position.y = self.path["y"][self.path_index]
				self.path_pub.publish(self.path_pose)

				self.count += 1

		self.quad_desired_state_msg.header.stamp = rospy.Time.now()
		self.quad_desired_state_msg.pose.z = self.setpoint_msg.pose.position.z
		self.quad_desired_state_pub.publish(self.quad_desired_state_msg)



	#handler of the mocap PoseStamped message
	def mocap_pose_callback(self, mocap_pose_msg):
		self.change_setpoint(mocap_pose_msg)

	# handler of the mocap TransformStamped message
	def mocap_transform_callback(self, mocap_transform_msg):
	#we want to use only PoseStamped message so we transform the TransforStamped message to some PoseStamped message
		mocap_pose_msg = PoseStamped()
		mocap_pose_msg.pose.position.x = mocap_transform_msg.transform.translation.x
		mocap_pose_msg.pose.position.y = mocap_transform_msg.transform.translation.y
		mocap_pose_msg.pose.position.z = mocap_transform_msg.transform.translation.z
		mocap_pose_msg.pose.orientation.x = mocap_transform_msg.transform.rotation.x
		mocap_pose_msg.pose.orientation.y = mocap_transform_msg.transform.rotation.y
		mocap_pose_msg.pose.orientation.z = mocap_transform_msg.transform.rotation.z
		mocap_pose_msg.pose.orientation.w = mocap_transform_msg.transform.rotation.w
		self.change_setpoint(mocap_pose_msg)

	def rc_raw_callback(self, rc_msg):
		if rc_msg.values[6] > 1500:
			self.not_activated = False
		else:
			self.not_activated = True

	#this function retrieve the information of one setpoint from setpoint dictionary and change their format to PoseStamped message. It also send back the rotating direction for every setpoint
	def pose_msg_from_dict(self,setpoint_dict):
		pose_msg = PoseStamped()
		euler_xyz = [0,0,np.deg2rad(setpoint_dict["yaw_degrees"])]
		q = tf.transformations.quaternion_from_euler(*euler_xyz,axes="rxyz")
		setpoint_rotating_direction = setpoint_dict["clockwise"]
		pose_msg.pose.position.x = setpoint_dict["position"][0]
		pose_msg.pose.position.y = setpoint_dict["position"][1]
		pose_msg.pose.position.z = setpoint_dict["position"][2]

		pose_msg.pose.orientation.x = q[0]
		pose_msg.pose.orientation.y = q[1]
		pose_msg.pose.orientation.z = q[2]
		pose_msg.pose.orientation.w = q[3]

		return pose_msg,setpoint_rotating_direction


	#this function write a message in the console when a new setpoint is applied
	def output_change_of_waypoint(self):

		info_str = "Requesting setpoint %d of %d: position (m): %s, yaw (degrees): %d, setpoint direction : %d (1 = clockwise, 0 = CCW), current entry point : %s" %(
			self.waypoint_index+1, len(self.waypoint_list),
			str(self.waypoint_list[self.waypoint_index]["position"]),
			self.waypoint_list[self.waypoint_index]["yaw_degrees"],
			self.waypoint_list[self.waypoint_index]["clockwise"],
			str(self.current_entry_point)

	)
		rospy.logwarn(info_str)



	#this function is call everytime a new current position is received. It will change the behaviour of the robot and set his different states
	def change_setpoint(self, mocap_pose_msg):
		#register the current position of the robot
		self.current_position =[mocap_pose_msg.pose.position.x,mocap_pose_msg.pose.position.y]
		#at the beginning of the application, the robot is set to go in straight line to the tangent point of the initialised setpoint
		if self.initialisation:
			#his state is straight
			self.STATE = "straight"
			self.entry_point_available = False
			#the entry point of the setpoint is computed thanks to the direction (CW or CCW), the current_position and the position of the set point
			self.current_entry_point = self.compute_tangent_setpoint_entry_point(self.setpoint_direction,
																				 mocap_pose_msg.pose.position.x,
																				 mocap_pose_msg.pose.position.y,
																				 self.setpoint_msg.pose.position.x,
																				 self.setpoint_msg.pose.position.y,
																				 self.setpoint_radius_tolerance,
																				 )
			current_pose = [mocap_pose_msg.pose.position.x,mocap_pose_msg.pose.position.y]
			#the current index of the path is set to zero
			self.path_index = 0
			#we compute the number of point of the path to go to the entry point with a straight line
			self.number_path_point = self.compute_number_of_point("straight",current_pose,self.current_entry_point,self.distance_between_point)
			#we register the different pose of the path to go to the entry point
			self.path = self.compute_line_path(current_pose,self.current_entry_point,self.number_path_point)
			#the first setpoint is set and the initialisation state is finished
			self.output_change_of_waypoint()
			if len(self.waypoint_list) == 1 :
				self.current_end_point = self.current_entry_point
				self.end = True

			self.initialisation = False

		# computation of the roll, pitch, yaw of the robot thanks to the mocap PoseStamped message (quaternion)
		orientation = [mocap_pose_msg.pose.orientation.x,mocap_pose_msg.pose.orientation.y,mocap_pose_msg.pose.orientation.z,mocap_pose_msg.pose.orientation.w]
		current_roll, current_pitch, current_yaw = tf.transformations.euler_from_quaternion(orientation)
		#computation of the distance between the current pose and the entry_point
		dist_to_entry_point = math.sqrt(math.pow((self.current_entry_point[1]-mocap_pose_msg.pose.position.y),2) + (math.pow((self.current_entry_point[0]-mocap_pose_msg.pose.position.x),2)))
		# computation of the distance between the current pose and the end_point
		dist_to_end_point = math.sqrt(math.pow((self.current_end_point[1]-mocap_pose_msg.pose.position.y),2) + (math.pow((self.current_end_point[0]-mocap_pose_msg.pose.position.x),2)))


		#if the robot is in a circular state and it has performed at least the half of the angular way that it must do. Then, the end point of the circular trajectory is available
		#this if statement is to make sure that the robot will do a full trajectory around the setpoint and won't leave it before it finished the trajectory
		if self.STATE == "circular":
			if (self.path_index >=  (self.number_path_point - 1)):
				self.end_point_available = True

		if self.STATE == "straight":
			if (self.path_index >=  (self.number_path_point - 1)):
				self.entry_point_available = True



		#when the robot reach the entry point of a setpoint and change his state to a circular state
		if self.STATE == "straight" and self.entry_point_available:
			setpoint_position = [self.setpoint_msg.pose.position.x, self.setpoint_msg.pose.position.y]
			#a new circular path is computed and the current path index is reset to 0
			self.path_index = 0
			self.dist_to_tangent_point = dist_to_end_point
			# we compute the number of point of the path to go to the end point with a circular line
			self.number_path_point = self.compute_number_of_point("circular",self.current_entry_point,self.current_end_point,self.distance_between_point,self.setpoint_radius_tolerance,setpoint_position,self.setpoint_direction )
			self.number_path_point = int(self.number_path_point * self.circular_velocity_ratio)
			# we register the different pose of the path to go to the end point
			self.path = self.compute_circle_path(self.setpoint_direction,self.current_entry_point,self.current_end_point,setpoint_position,self.setpoint_radius_tolerance,self.number_path_point)
			if self.end == True:
				self.last_circular_path = self.path.copy()
			#the robot state is set to circular
			self.STATE = "circular"
			#the end point isn t available until the robot has performed half of his angular path
			self.end_point_available = False
			#we register the previous velocity at the transition (from straight to circular state) in order to have a smoother acceleration
			self.initial_straight_velocity = self.setvelocity_msg.twist.linear.x
			self.initial_angular_velocity = self.setvelocity_msg.twist.angular.z


		# when the robot reach the entry point of a setpoint and change his state to a straight state
		if self.STATE == "circular" and self.end_point_available:

			if  not(self.end):
				# the robot state is set to circular
				self.STATE = "straight"
				self.entry_point_available = False
				# we register the previous velocity at the transition (from circular to straight) in order to have a smoother acceleration
				self.initial_straight_velocity = self.setvelocity_msg.twist.linear.x
				self.initial_angular_velocity = self.setvelocity_msg.twist.angular.z
				self.dist_to_tangent_point = dist_to_entry_point

				#if there is still some setpoint left
				if self.waypoint_index < len(self.waypoint_list) :
					self.waypoint_index += 1
					#register the new setpoint position and rotation direction
					self.setpoint_msg, self.setpoint_direction = self.pose_msg_from_dict(self.waypoint_list[self.waypoint_index])

					#compute the new entry point of the new setpoint
					self.current_entry_point = self.compute_tangent_setpoint_entry_point(self.setpoint_direction, mocap_pose_msg.pose.position.x,
																						 mocap_pose_msg.pose.position.y,
																						 self.setpoint_msg.pose.position.x,
																						 self.setpoint_msg.pose.position.y,
																						 self.setpoint_radius_tolerance,
																						 )
					current_pose = [mocap_pose_msg.pose.position.x, mocap_pose_msg.pose.position.y]
					# a new straight path is computed and the current path index is reset to 0
					self.path_index = 0
					# we compute the the path to go to the entry point with a straight line
					self.number_path_point = self.compute_number_of_point("straight",current_pose,self.current_entry_point,self.distance_between_point)
					self.path = self.compute_line_path(current_pose, self.current_entry_point, self.number_path_point)
					self.output_change_of_waypoint()
					#if there are still 2 setpoint left compute the end point of the new setpoint
					if self.waypoint_index < len(self.waypoint_list) - 1:
						self.next_setpoint_msg, self.next_setpoint_direction = self.pose_msg_from_dict(self.waypoint_list[self.waypoint_index + 1])
						#if the direction of the new setpoint is CW then we compute the tangent end point of the new setpoint thanks to the position of the next setpoint and inverted direction of the new setpoint
						if self.setpoint_direction == 0:
							self.current_end_point = self.compute_tangent_setpoint_entry_point(1,
																							   self.next_setpoint_msg.pose.position.x,
																							   self.next_setpoint_msg.pose.position.y,
																							   self.setpoint_msg.pose.position.x,
																							   self.setpoint_msg.pose.position.y,
																							   self.setpoint_radius_tolerance)
						elif self.setpoint_direction == 1:
							self.current_end_point = self.compute_tangent_setpoint_entry_point(0,
																							   self.next_setpoint_msg.pose.position.x,
																							   self.next_setpoint_msg.pose.position.y,
																							   self.setpoint_msg.pose.position.x,
																							   self.setpoint_msg.pose.position.y,
																							   self.setpoint_radius_tolerance)

					else:
						#if there is only one setpoint left, the dubins s path stop because there are no trajectory around the setpoint to compute. Then the end point is equal to the entry point because the robot is supposed to turn around the last setpoint
						self.current_end_point = self.current_entry_point
						self.end = True

			else:
				#When the robot has finished his first circle around the last checkpoint, we add one more circle path around the last setpoint.
				self.end_point_available = False
				self.infinite_finish_loop = True
				#self.last_initial_path_index = self.path_index
				self.path["x"]+=self.last_circular_path["x"]
				self.path["y"]+=self.last_circular_path["y"]
				self.path["angle"]+=self.last_circular_path["angle"]
				self.number_path_point = len(self.path["x"])




		current_position = [mocap_pose_msg.pose.position.x,mocap_pose_msg.pose.position.y]
		goal_position = [self.path["x"][self.path_index],self.path["y"][self.path_index]]
		#computation of the linear and angular velocity thanks to the current position, the state of the robot and the current path position
		linear_velocity, angular_velocity = self.compute_velocity(current_position,goal_position,current_yaw,self.Kx,self.Ky,self.Ktheta,self.path,self.path_index,self.path_rate,self.setpoint_radius_tolerance)

		#if the computed angular velocity is too high or too low then set the angular velocity to PhiMax or -PhiMax
		#the angular velocity is computed in NED frame !!!!!!!!!
		if angular_velocity > self.PhiMax :
			self.setvelocity_msg.twist.angular.z = self.robot_command_orientation * self.PhiMax
		elif angular_velocity < -self.PhiMax:
			self.setvelocity_msg.twist.angular.z = self.robot_command_orientation*(-self.PhiMax)
		else:
			self.setvelocity_msg.twist.angular.z = self.robot_command_orientation * angular_velocity

		# if the computed linear velocity is too high or too low then set the linear velocity to VMax or Vmin
		if linear_velocity < self.VMin:
			self.setvelocity_msg.twist.linear.x = self.VMin
		elif linear_velocity > self.VMax:
			self.setvelocity_msg.twist.linear.x = self.VMax
		else:
			self.setvelocity_msg.twist.linear.x = linear_velocity


	#This function compute the tangent entry point of a setpoint relatively to the current position of the robot and his rotating direction around the setpoint
	def compute_tangent_setpoint_entry_point(self,in_out, current_x,current_y,setpoint_x, setpoint_y, R):
		#computation of distance between the current position and center of the setpoint
		dist_from_setpoint = math.sqrt(math.pow(current_x - setpoint_x, 2) + math.pow(current_y - setpoint_y, 2))
		#computation of distance between the current position and the tangents point
		r1 = math.sqrt(math.pow(dist_from_setpoint, 2) - math.pow(R, 2))
		#computation of the pose of the 2 tangents points thanks to the intersection between the setpoint circle and the circle with a radius of the previous computed distance
		pe1, pe2 = self.circle_intersection(current_x, current_y , r1 , setpoint_x, setpoint_y, R)
		#there is 2 solutions to the circle intersection, we choose the right thanks to the rotating direction around the setpoint
		if in_out == 1:
			pe = pe1
		elif in_out == 0:
			pe = pe2
		return pe

	#resolution of the 2nd degree equation to find the 2 poses of circles intersection
	def circle_intersection(self, x1, y1, r1, x2, y2, r2):
		dx, dy = x2 - x1, y2 - y1
		d = math.sqrt(dx * dx + dy * dy)
		a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
		h = math.sqrt(r1 * r1 - a * a)
		xm = x1 + a * dx / d
		ym = y1 + a * dy / d
		xs1 = xm + h * dy / d
		xs2 = xm - h * dy / d
		ys1 = ym - h * dx / d
		ys2 = ym + h * dx / d
		return [xs1, ys1], [xs2, ys2]

	# computation of the straight line coefficient
	def compute_straight_line_coefficient(self,current_position, goal_position):
		x1, y1 = current_position
		x2, y2 = goal_position
		a = (y2 - y1)/(x2-x1)
		b = y2 - (a * x2)
		return a,b



	#computation of a straight line path between a current position and a goal position with the number of points as argument
	def compute_line_path(self,current_position, goal_position, number_path_point):
		path = {}
		x1, y1 = current_position
		x2, y2 = goal_position
		path["type"] = "straight"
		path["x"] = []
		path["y"] = []
		#x_distrib = np.linspace(-6, 6, num = number_path_point)
		for i in range(number_path_point):
			#computation of all the poses
			x = x1 + ((x2-x1)/number_path_point)* (i+1)
			y = y1 + ((y2-y1)/number_path_point)* (i+1)
			path["x"].append(x)
			path["y"].append(y)
		return path





	#computation of a circular line path between a start position and a end position on a circle with the number of points, the center of the circle, the radius of the circle and the rotatin direction as arguments
	def compute_circle_path(self, direction, start_position,end_position, circle_center, R , number_path_point):
		#initialisation of the variables
		path = {}
		x_s, y_s = start_position
		x_c, y_c = circle_center
		x_e, y_e = end_position


		#First of all we compute the distance from the start point relatively to the circle's center in x and y
		err_start_x = (x_s-x_c)
		err_start_y = (y_s - y_c)
		# then we compute the distance from the end point relatively to the center in x and y
		err_end_x = (x_e-x_c)
		err_end_y = (y_e - y_c)

		# we can now compute the angle of the start position relatively to the center of the circle
		if err_start_x > 0 and err_start_y >= 0:
			relative_start_angle = math.atan(err_start_y / err_start_x)
		elif err_start_x > 0 and err_start_y < 0:
			relative_start_angle = math.atan(err_start_y / err_start_x) + 2 * math.pi
		elif err_start_x < 0:
			relative_start_angle = math.atan(err_start_y / err_start_x) + math.pi
		elif err_start_x == 0 and err_start_y > 0:
			relative_start_angle = math.pi / 2
		elif err_start_x == 0 and err_start_y < 0:
			relative_start_angle = 3 * math.pi / 2

		if x_s != x_e and y_s != y_e:

			# we compute the angle of the end position relatively to the center of the circle
			if err_end_x > 0 and err_end_y  >= 0:
				relative_end_angle = math.atan(err_end_y/err_end_x)
			elif err_end_x  > 0 and err_end_y < 0:
				relative_end_angle = math.atan(err_end_y/err_end_x) + 2*math.pi
			elif err_end_x < 0 :
				relative_end_angle = math.atan(err_end_y/err_end_x) + math.pi
			elif err_end_x == 0 and err_end_y > 0:
				relative_end_angle = math.pi/2
			elif err_end_x == 0 and err_end_y < 0:
				relative_end_angle = 3*math.pi/2

			#we can now compute the error between these two angles in order to know the angle between the start position and the end position
			#It most be inferior to 2 * Pi
			relative_full_angle = relative_end_angle - relative_start_angle
			relative_full_angle = relative_full_angle % (2 * math.pi)

			#if the direction is CCW then we take the opposite angle
			if direction == 0 :
				relative_full_angle = relative_full_angle - (2 * math.pi)
		elif x_s == x_e and y_s == y_e:
			#if the end position == the start position then we assume that we want to compute a full circle trajectory path around the setpoint
			relative_full_angle = 2 * math.pi
			if direction == 0 :
				relative_full_angle = - (2 * math.pi)


		#now we have the full angle between the 2 positions
		# As we know, the number of point of our trajectory around the circlem we can compute the angle step between each point
		angle_step = relative_full_angle/number_path_point
		path["type"] = "circular"
		path["x"] = []
		path["y"] = []
		path["angle"] = []
		path["angle_step"] = angle_step

		#Then we can compute every path's pose around the circle
		for i in range(number_path_point):
			angle = relative_start_angle + (angle_step*(i+1))
			x = x_c + R * math.cos(angle)
			y = y_c + R * math.sin(angle)
			path["x"].append(x)
			path["y"].append(y)
			path["angle"].append(angle)
			if (i == int((number_path_point-1)/2)):
				path["middle_angle"] = angle
		return path


	#computation of the velocity applied to the robot thanks to his current position and orientation, his goal position, gains
	def compute_velocity(self, current_position,goal_position,current_yaw,Kx,Ky,Ktheta, path, current_path_index, rate, R):
		#initialisation of the variables
		x, y = current_position
		x_r, y_r = goal_position
		delta_t = 1/rate
		#computation of the error between the current position and the goal position in x and y
		err_x = x - x_r
		err_y = y - y_r

		#computation of the derivative term of the controller respectively to the robot trajectory type
		if path["type"] == "straight":
			if current_path_index >=  len(path["x"])-1:
				#if the path is not evolving anymore
				derivative_x_r = 0
				derivative_y_r = 0
			else:
				derivative_x_r = (path["x"][current_path_index+1] - path["x"][current_path_index])/delta_t
				derivative_y_r = (path["y"][current_path_index+1] - path["y"][current_path_index])/delta_t
			x_computed = -Ky * err_x + derivative_x_r
			y_computed = -Kx * err_y + derivative_y_r
		elif path["type"] == "circular":
			if current_path_index >= len(path["x"])-1:
				derivative_x_r = 0
				derivative_y_r = 0
			else:
				derivative_x_r = R * (-math.sin(path["angle"][current_path_index])) * (path["angle_step"]/delta_t)
				derivative_y_r = R * (math.cos(path["angle"][current_path_index])) * (path["angle_step"]/delta_t)

			x_computed = -Kx * err_x + derivative_x_r
			y_computed = -Ky * err_y + derivative_y_r

		#computation of angle between the robot and the goal position
		theta_computed = math.atan2(y_computed, x_computed)
		#then we can compute the angle that the robot has to perform to face the goal position ( robot orientation - angle_between_robot_goal_position)
		theta_err = current_yaw - theta_computed



		#the linear velocity is computed thanks to the angle that the robot has to perform and the distance between it and the goal position.
		linear_vel = math.sqrt(x_computed * x_computed + y_computed * y_computed) * math.cos(theta_err)

		# if the robot has to perform an angle superior to PI, we ask him to go in the opposite rotation way
		if (theta_err > math.pi):
			theta_err = -(2 * math.pi - theta_err)

		if (theta_err < -math.pi):
			theta_err = theta_err + 2 * math.pi

		#we add the gain on the angular velocity
		angular_vel = -Ktheta * theta_err


		return linear_vel, angular_vel

	#this function compute the number of point between 2 positions thanks to the type of trajectory and the distance between each point
	def compute_number_of_point(self,trajectory_type, current_position,goal_position,distance_between_point, R = None, center_position = None, direction = None):
			# initialisation of the variables
			x_s,y_s = current_position
			x_e, y_e = goal_position

			#if the trajectory is straight just divide the distance between the 2 position by the distance between each point
			if trajectory_type == "straight":
				distance = math.sqrt((y_e-y_s)* (y_e-y_s) +(x_e-x_s)* (x_e-x_s) )
				number_of_point = distance/ distance_between_point
			elif trajectory_type == "circular" and x_s != x_e and y_s != y_e:
				x_c, y_c = center_position
				# First of all we compute the distance from the start point relatively to the circle's center in x and y
				err_start_x = (x_s - x_c)
				err_start_y = (y_s - y_c)
				# then we compute the distance from the end point relatively to the center in x and y
				err_end_x = (x_e - x_c)
				err_end_y = (y_e - y_c)
				# we can now compute the angle of the start position relatively to the center of the circle
				if err_start_x > 0 and err_start_y >= 0:
					relative_start_angle = math.atan(err_start_y / err_start_x)
				elif err_start_x > 0 and err_start_y < 0:
					relative_start_angle = math.atan(err_start_y / err_start_x) + 2 * math.pi
				elif err_start_x < 0:
					relative_start_angle = math.atan(err_start_y / err_start_x) + math.pi
				elif err_start_x == 0 and err_start_y > 0:
					relative_start_angle = math.pi / 2
				elif err_start_x == 0 and err_start_y < 0:
					relative_start_angle = 3 * math.pi / 2
				# we compute the angle of the end position relatively to the center of the circle
				if err_end_x > 0 and err_end_y >= 0:
					relative_end_angle = math.atan(err_end_y / err_end_x)
				elif err_end_x > 0 and err_end_y < 0:
					relative_end_angle = math.atan(err_end_y / err_end_x) + 2 * math.pi
				elif err_end_x < 0:
					relative_end_angle = math.atan(err_end_y / err_end_x) + math.pi
				elif err_end_x == 0 and err_end_y > 0:
					relative_end_angle = math.pi / 2
				elif err_end_x == 0 and err_end_y < 0:
					relative_end_angle = 3 * math.pi / 2
				# we can now compute the error between these two angles in order to know the angle between the start position and the end position
				# It most be inferior to 2 * Pi
				relative_full_angle = relative_end_angle - relative_start_angle
				relative_full_angle = relative_full_angle % (2 * math.pi)
				# if the direction is CCW then we take the opposite angle
				if direction == 0:
					relative_full_angle = relative_full_angle - (2 * math.pi)
				#Then we compute the proportion of this performed angle relatively to the full circle angle
				proportion_of_circle = math.fabs(relative_full_angle) / (2 * math.pi)
				#finally, we compute the distance performed on the circle thanks to the circumference of the circle and the proportion previously computed
				distance = (2 * math.pi * R) * proportion_of_circle
				#then we divide this distance by the distance between each point
				number_of_point = distance / distance_between_point
			elif trajectory_type == "circular" and x_s == x_e and y_s == y_e:
				# if the end position == the start position then we assume that we want to compute a full circle trajectory path around the setpoint
				distance = (2 * math.pi * R)
				number_of_point = distance / distance_between_point

			return int(number_of_point)
