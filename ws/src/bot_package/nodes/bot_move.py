from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from nodes.bot_functions import BotMethods
from nodes.read_methods import ReadMethods

import copy

class PublisherJointTrajectory(Node):
	def __init__(self):
		super().__init__("publisher_position_trajectory_controller")
		# Declare all parameters
		self.declare_parameter("controller_name", "position_trajectory_controller")
		self.declare_parameter("wait_sec_between_publish", 6)
		self.declare_parameter("goal_names", ["pos1", "pos2"])
		self.declare_parameter("joints", [""])
		self.declare_parameter("check_starting_point", False)

		# Read parameters
		controller_name = self.get_parameter("controller_name").value
		wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
		goal_names = self.get_parameter("goal_names").value
		self.joints = self.get_parameter("joints").value
		self.check_starting_point = self.get_parameter("check_starting_point").value
		self.starting_point = {}

		if self.joints is None or len(self.joints) == 0:
			raise Exception('"joints" parameter is not set!')

		# starting point stuff
		if self.check_starting_point:
			# declare nested params
			for name in self.joints:
				param_name_tmp = "starting_point_limits" + "." + name
				self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
				self.starting_point[name] = self.get_parameter(param_name_tmp).value

			for name in self.joints:
				if len(self.starting_point[name]) != 2:
					raise Exception('"starting_point" parameter is not set correctly!')
			self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)

		# initialize starting point status
		self.starting_point_ok = not self.check_starting_point

		self.joint_state_msg_received = False

		# Read all positions from parameters
		# Dict of JointTrajectoryPoint
		self.goals = ReadMethods.read_positions_from_parameters(self, goal_names)

		self.goal_names = list(self.goals.keys())

		if len(self.goals) < 1:
			self.get_logger().error("No valid goal found. Exiting...")
			exit(1)

		publish_topic = "/" + controller_name + "/" + "joint_trajectory"

		# self.get_logger().info(f'Publishing {len(goal_names)} goals on topic "{publish_topic}" every "{wait_sec_between_publish} s"')
		self.get_logger().info(f'\n\tPublishing {len(goal_names)}...\n')

		self._publisher = self.create_publisher(JointTrajectory, publish_topic, 1)
		self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
		self.i = 0

		# Count for checking start position
		self.count = 0

	def timer_callback(self):
		if self.starting_point_ok:
			# goal = self.goal_names[self.i]

			# Return trajectories to move from home to chip 1
			if self.i < 7:
				trajectories = BotMethods.move_chip(self, self.joints, self.goals, 1)
				traj = trajectories[self.i]
				self.get_logger().info(f'trajectory_number: {str(self.i)}')
				traj_goal = traj.points[0]

				# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
				pos = f'[Base: {traj_goal.positions[0]}, Shoulder: {traj_goal.positions[1]}, Elbow: {traj_goal.positions[2]}, ' +\
				f'Wrist 1: {traj_goal.positions[3]}, Wrist 2: {traj_goal.positions[4]}, Wrist 3: {traj_goal.positions[5]}]'

				# Using goals as dict type
				self.get_logger().info(f"Sending goal:\n\t{pos}.\n")

				#region KEEPME
				# traj = JointTrajectory()
				# traj.joint_names = self.joints
				# # traj.points.append(self.goals[self.i])
				# traj.points.append(self.goals[goal]) # Using goals as dict type
				#endregion KEEPME

				self._publisher.publish(traj)

				self.i += 1
				# self.i %= len(trajectories)

		elif self.check_starting_point and not self.joint_state_msg_received:
			self.get_logger().warn('Start configuration could not be checked! Check "joint_state" topic!')
		# else:
		# 	self.get_logger().warn("Start configuration is not within configured limits!")
		else:
			self.get_logger().warn("Moving back to preferred position.")
			temp = JointTrajectory()
			temp.joint_names = self.joints
			temp_traj = []

			temp.points.append(self.goals["safe_start"])
			temp_traj.append(copy.deepcopy(temp))
			temp.points.clear()

			temp.points.append(self.goals["home"])
			temp_traj.append(copy.deepcopy(temp))
			temp.points.clear()

			temp_pos = temp_traj[self.count]

			self._publisher.publish(temp_pos)

			self.count += 1


	def joint_state_callback(self, msg):
		if not self.joint_state_msg_received:
			# check start state
			limit_exceeded = [False] * len(msg.name)
			for idx, enum in enumerate(msg.name):
				if (msg.position[idx] < self.starting_point[enum][0]) or (msg.position[idx] > self.starting_point[enum][1]):
					self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
					limit_exceeded[idx] = True

			if any(limit_exceeded):
				self.starting_point_ok = False
			else:
				self.starting_point_ok = True

			self.joint_state_msg_received = True
		else:
			return
