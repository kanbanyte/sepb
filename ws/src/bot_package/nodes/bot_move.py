import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from ur_dashboard_msgs.srv import Load
from sensor_msgs.msg import JointState
from nodes.bot_functions import BotMethods
from nodes.read_methods import ReadMethods

from pick_place_interfaces.srv import PickPlaceService

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
		self.wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
		goal_names = self.get_parameter("goal_names").value
		self.joints = self.get_parameter("joints").value
		self.check_starting_point = self.get_parameter("check_starting_point").value
		self.starting_point = {}

		self.subnode = rclpy.create_node('subnode')

		# Create client for case, chip, and tray services
		self.case_cli = self.subnode.create_client(PickPlaceService, 'case')
		self.chip_cli = self.subnode.create_client(PickPlaceService, 'chip')
		self.tray_cli = self.subnode.create_client(PickPlaceService, 'tray')

		self.load_program_cli = self.create_client(Load, '/dashboard_client/load_program')
		# self.load_program_cli.call()

		# TODO: Replace Case.srv, Chip.srv, and Tray.srv with one .srv file because all take request: bool, response: int64
		while not self.case_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.case_cli.srv_name} service not available, trying again...")
		self.request = PickPlaceService.Request()

		while not self.chip_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.chip_cli.srv_name} service not available, trying again...")
		self.request = PickPlaceService.Request()

		while not self.tray_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.chip_cli.srv_name} service not available, trying again...")
		self.request = PickPlaceService.Request()

		if self.joints is None or len(self.joints) == 0:
			raise Exception('"joints" parameter is not set!')

		publish_topic = "/" + controller_name + "/" + "joint_trajectory"

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
			self.joint_state_pub = self.create_publisher(JointTrajectory, publish_topic, 1)
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

		self.get_logger().info(f'\n\tPublishing {len(goal_names)} goals every {self.wait_sec_between_publish} secs...\n')

		# Get list of all trajectories to move to
		# Args: joints, goals, chip_number, case_number, tray_number
		# self.trajectories = BotMethods.get_all_trajectories(self.joints, self.goals, 24, 1, 2)
		self.trajectory_names = BotMethods.get_trajectory_names()

		# Trajectories to move the cobot to a safe position and back to home
		self.safe_start_trajectories = [JointTrajectory(), JointTrajectory()]

		self.safe_start_trajectories[0].joint_names = self.joints
		self.safe_start_trajectories[1].joint_names = self.joints

		self.safe_start_trajectories[0].points.append(self.goals["safe_start"])
		self.safe_start_trajectories[1].points.append(self.goals["home"])

		self.trajectories = self.populate_trajectories()

		self._publisher = self.create_publisher(JointTrajectory, publish_topic, 1)
		self.timer = self.create_timer(self.wait_sec_between_publish, self.timer_callback)
		self.i = 0

	def send_request(self, service, detect):
		self.request.detect = detect
		self.future = service.call_async(self.request)
		rclpy.spin_until_future_complete(self, self.future)

		# return self.future.result()
		return self.future

	def populate_trajectories(self):
		future = self.send_request(self.chip_cli, True)
		rclpy.spin_until_future_complete(self.subnode, future)

		# chip_position = self.send_request(self.chip_cli, True)
		# case_position = self.send_request(self.case_cli, True)
		# tray_position = self.send_request(self.tray_cli, True)

		# while chip_position.signal == -1:
		# 	chip_position = self.send_request(self.chip_cli, True)
		# while case_position.signal == -1:
		# 	case_position = self.send_request(self.case_cli, True)
		# while tray_position.signal == -1:
		# 	tray_position = self.send_request(self.tray_cli, True)

		if future.result() is not None:
			result = future.result()
			chip_position = result.signal
			self.get_logger().info(f"Populated trajectories: True")

			return BotMethods.get_all_trajectories(self.joints, self.goals, chip_position.signal, 1, 2)
		else:
			self.get_logger().error(f"Error when populating trajectories: {future.exception()}")
			return

		# return BotMethods.get_all_trajectories(self.joints, self.goals, chip_position.signal, case_position.signal, 2)

	# TODO: Find a way to restart after all trajectories have been moved through
	def timer_callback(self):
		if self.starting_point_ok:
			# Return trajectories to move from home to chip 1
			if self.i < len(self.trajectories):
				traj = self.trajectories[self.i]
				traj_name = self.trajectory_names[self.i]
				self.get_logger().info(f'Goal Name: {traj_name}')
				traj_goal = traj.points[0]

				# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
				pos = f'[Base: {traj_goal.positions[0]}, Shoulder: {traj_goal.positions[1]}, Elbow: {traj_goal.positions[2]}, ' +\
				f'Wrist 1: {traj_goal.positions[3]}, Wrist 2: {traj_goal.positions[4]}, Wrist 3: {traj_goal.positions[5]}] Velocity: {traj_goal.velocities}'

				# Using goals as dict type
				self.get_logger().info(f"Sending goal:\n\t{pos}.\n")

				self._publisher.publish(traj)

				self.i += 1
			else:
				self.get_logger().info("Pick and place task complete. Recalculating trajectories...")
				self.i = 0
				self.trajectories = self.populate_trajectories()
				self.timer.reset()

		elif self.check_starting_point and not self.joint_state_msg_received:
			self.get_logger().warn('Start configuration could not be checked! Check "joint_state" topic!')

	def joint_state_callback(self, msg):
		if not self.joint_state_msg_received:
			# check start state
			limit_exceeded = [False] * len(msg.name)
			for idx, enum in enumerate(msg.name):
				if (msg.position[idx] < self.starting_point[enum][0]) or (msg.position[idx] > self.starting_point[enum][1]):
					self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
					limit_exceeded[idx] = True

			if any(limit_exceeded):
				self.get_logger().warn("Moving back to preferred position.")
				temp = JointTrajectory()
				temp.joint_names = self.joints
				temp_traj = []

				temp.points.append(self.goals["safe_start"])
				temp_traj.append(copy.deepcopy(temp))
				temp.points.clear()

				for traj in temp_traj:
					temp_pos = traj
					self.joint_state_pub.publish(temp_pos)

				self.starting_point_ok = True
			else:
				self.starting_point_ok = True

			self.joint_state_msg_received = True
		else:
			return