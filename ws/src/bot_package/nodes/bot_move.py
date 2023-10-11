import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from nodes.bot_functions import BotMethods
from nodes.read_methods import ReadMethods

from pick_place_interfaces.srv import PickPlaceService
from pick_place_interfaces.action import PickPlaceAction

import copy
import time

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

		# Create action server
		self.action_server = ActionServer(self, PickPlaceAction, 'perform_pick_place', self.action_callback)

		# Create client for case, chip, and tray services
		self.case_cli = self.subnode.create_client(PickPlaceService, 'case')
		self.chip_cli = self.subnode.create_client(PickPlaceService, 'chip')
		self.tray_cli = self.subnode.create_client(PickPlaceService, 'tray')

		# TODO: Replace Case.srv, Chip.srv, and Tray.srv with one .srv file because all take request: bool, response: int64
		while not self.case_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.case_cli.srv_name} service not available, trying again...")

		while not self.chip_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.chip_cli.srv_name} service not available, trying again...")

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

		self.current_movement = "home"

		self._publisher = self.create_publisher(JointTrajectory, publish_topic, 1)
		# self.timer = self.create_timer(self.wait_sec_between_publish, self.timer_callback)
		self.i = 0

	def send_request(self, service, detect):
		self.request.detect = detect
		self.future = service.call_async(self.request)
		# rclpy.spin_until_future_complete(self, self.future)

		# return self.future.result()
		return self.future

	def populate_trajectories(self):
		future_chip = self.send_request(self.chip_cli, True)
		rclpy.spin_until_future_complete(self.subnode, future_chip)

		future_case = self.send_request(self.case_cli, True)
		rclpy.spin_until_future_complete(self.subnode, future_case)

		future_tray = self.send_request(self.tray_cli, True)
		rclpy.spin_until_future_complete(self.subnode, future_tray)

		chip_position = self.send_request(self.chip_cli, True)
		case_position = self.send_request(self.case_cli, True)
		tray_position = self.send_request(self.tray_cli, True)
		# if chip_position.result() is None:
		# 	self.get_logger().error(f"chip result is none: {chip_position.exception()}")
		# else:
		# 	self.get_logger().info('chip result is good.')
		# 	if chip_position.result().signal is None:
		# 		self.get_logger().error(f"chip signal result is none: {chip_position.exception()}")
		# 	else:
		# 		self.get_logger().info('chip signal result is good.')
		# self.get_logger().info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!{chip_position}")

		while chip_position.result() and chip_position.result().signal == -1:
			chip_position = self.send_request(self.chip_cli, True)
			self.get_logger().info('Waiting for chip signal to become available...')

		while case_position.result() and case_position.result().signal == -1:
			case_position = self.send_request(self.case_cli, True)
			self.get_logger().info('Waiting for case signal to become available...')

		while tray_position.result() and tray_position.result().signal == -1:
			tray_position = self.send_request(self.tray_cli, True)
			self.get_logger().info('Waiting for tray signal to become available...')

		if future_chip.result() and future_case.result() and future_tray.result() is not None :
			chip_result = future_chip.result()
			chip_position = chip_result.signal

			case_result = future_case.result()
			case_position = case_result.signal

			tray_result = future_tray.result()
			tray_position = tray_result.signal
			self.get_logger().info(f"Populating trajectories...")

			return BotMethods.get_all_trajectories(self.joints, self.goals, chip_position, case_position, 2)
		else:
			self.get_logger().error(f"Error when populating trajectories: {future_chip.exception()}")

		# return BotMethods.get_all_trajectories(self.joints, self.goals, chip_position.signal, case_position.signal, 2)

	def action_callback(self, goal_handle):
		self.get_logger().info("Executing pick and place task...")

		feedback_msg = PickPlaceAction.Feedback()
		feedback_msg.current_movement = self.current_movement

		self.trajectories = self.populate_trajectories()
		self.trajectory_names = BotMethods.get_trajectory_names()

		if self.starting_point_ok:
			for i in range(len(self.trajectories)):
				traj = self.trajectories[i]
				# traj_name = self.trajectory_names[i]
				feedback_msg.current_movement = self.trajectory_names[i]
				self.get_logger().info(f'Goal Name: {feedback_msg.current_movement}')
				traj_goal = traj.points[0]

				# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
				pos = f'[Base: {traj_goal.positions[0]}, Shoulder: {traj_goal.positions[1]}, Elbow: {traj_goal.positions[2]}, ' +\
				f'Wrist 1: {traj_goal.positions[3]}, Wrist 2: {traj_goal.positions[4]}, Wrist 3: {traj_goal.positions[5]}]'

				# Using goals as dict type
				self.get_logger().info(f"Sending goal:\n\t{pos}.\n")

				self._publisher.publish(traj)

				# Wait for number of seconds defined in config file
				time.sleep(self.wait_sec_between_publish)

			goal_handle.succeed()
			self.get_logger().info("Pick and place task complete.")
			result = PickPlaceAction.Result()
			result.task_successful = True

			return result

		elif self.check_starting_point and not self.joint_state_msg_received:
			self.get_logger().warn('Start configuration could not be checked! Check "joint_state" topic!')

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
