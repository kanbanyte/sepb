import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from nodes.cobot_methods import get_tray_movement_trajectories, get_all_trajectories
from nodes.read_methods import read_positions_from_parameters, read_gripper_outputs_from_parameters
from data_processing.tray_position import CobotMovement

from pick_place_interfaces.srv import PickPlaceService
from pick_place_interfaces.action import PickPlaceAction

import copy
import time

class CobotMovementActionServer(Node):
	def __init__(self):
		super().__init__("cobot_movement_node")
		# Declare all parameters
		self.declare_parameter("controller_name", "position_trajectory_controller")
		self.declare_parameter("wait_sec_between_publish", 6)
		self.declare_parameter("goal_names", ["pos1", "pos2"])
		self.declare_parameter("joints", [""])
		self.declare_parameter("check_starting_point", False)
		self.declare_parameter("gripper_outputs", [""])

		# Read parameters
		controller_name = self.get_parameter("controller_name").value
		self.wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
		goal_names = self.get_parameter("goal_names").value
		self.joints = self.get_parameter("joints").value
		self.check_starting_point = self.get_parameter("check_starting_point").value
		self.starting_point = {}
		gripper_output_names = self.get_parameter("gripper_outputs").value

		self.subnode = rclpy.create_node('subnode')

		# Create action server and hook the callback
		self.action_server = ActionServer(self, PickPlaceAction, 'perform_pick_place', self.execute_pick_and_place)
		# self.action_server = ActionServer(self, PickPlaceAction, 'perform_pick_place', self.execute_test)

		# Create client for the following services: gripper, case detection, chip detection, tray detection
		self.gripper_cli = self.subnode.create_client(SetIO, 'gripper_service')
		self.case_cli = self.subnode.create_client(PickPlaceService, 'case')
		self.chip_cli = self.subnode.create_client(PickPlaceService, 'chip')
		self.tray_cli = self.subnode.create_client(PickPlaceService, 'tray')

		# Ensuring all four services are connected
		while not self.gripper_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.gripper_cli.srv_name} service not available, trying again in 10s...")

		while not self.case_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.case_cli.srv_name} service not available, trying again in 10s...")

		while not self.chip_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.chip_cli.srv_name} service not available, trying again in 10s...")

		while not self.tray_cli.wait_for_service(timeout_sec=10.0):
			self.get_logger().info(f"{self.tray_cli.srv_name} service not available, trying again in 10s...")

		self.pick_place_request = PickPlaceService.Request()
		self.gripper_request = SetIO.Request()

		# Set fun and state for gripper request.
		# State is always 1.0 because gripper program resets state after it is called.
		self.gripper_request.fun = 1
		self.gripper_request.state = 1.0

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
		self.goals = read_positions_from_parameters(self, goal_names)
		self.goal_names = list(self.goals.keys())

		# Read all gripper outputs from parameters
		# Dict of int
		self.gripper_outputs = read_gripper_outputs_from_parameters(self, gripper_output_names)
		self.gripper_output_names = list(self.gripper_outputs.keys())

		if len(self.goals) < 1:
			self.get_logger().fatal("No valid goal found. Exiting...")
			exit(1)

		# always move back to the home position when started
		self.current_movement = "home"

		self._publisher = self.create_publisher(JointTrajectory, publish_topic, 1)

	def execute_test(self, goal_handle):
		'''
		Callback to test the server without moving the cobot.

		This function simulates a task execution without actually moving the cobot, providing a testing environment for the server and client.

		Args:
			goal_handle (actionlib.action_server.SimpleGoalHandle): the goal handle associated with the action request.

		Returns:
        	PickPlaceAction.Result: The result of the testing task.
		'''
		self.get_logger().info("")
		self.get_logger().info("Executing testing task...")

		result = PickPlaceAction.Result()

		feedback_msg = PickPlaceAction.Feedback()
		feedback_msg.current_movement = self.current_movement

		if not self.starting_point_ok:
			if self.check_starting_point and not self.joint_state_msg_received:
				self.get_logger().warn('Start configuration could not be checked! Check "joint_state" topic!')

		self.get_logger().info("Action in progress for 5s...")
		time.sleep(5)
		goal_handle.succeed()
		self.get_logger().info("Test action complete.")
		result.task_successful = True

		return result

	def execute_pick_and_place(self, goal_handle):
		'''
		Callback to execute a pick-and-place task.
		This function either loads a tray or moves it, depending on the suggested move from the tray service.
		Since only one action is performed by the callback, the client might need to send multiple requests to complete a cycle or
		keep the cobot performing in a loop.

		Args:
			goal_handle (actionlib.action_server.SimpleGoalHandle): the goal handle associated with the action request.

		Returns:
        	PickPlaceAction.Result: The result of the testing task.
		'''

		self.get_logger().info("Executing pick and place task...")

		result = PickPlaceAction.Result()

		feedback_msg = PickPlaceAction.Feedback()
		feedback_msg.current_movement = self.current_movement

		if not self.starting_point_ok:
			if self.check_starting_point and not self.joint_state_msg_received:
				self.get_logger().warn('Start configuration could not be checked! Check "joint_state" topic!')

			goal_handle.abort()
			result.task_successful = False
			return result

		cobot_move_signal = self.get_tray_signal()
		if (cobot_move_signal == CobotMovement.ASSEMBLY_TO_TRAY1.value or
	   		cobot_move_signal == CobotMovement.ASSEMBLY_TO_TRAY2.value or
			cobot_move_signal == CobotMovement.TRAY1_TO_ASSEMBLY.value or
			cobot_move_signal == CobotMovement.TRAY2_TO_ASSEMBLY.value):

			move_succeeded = self.move_tray(goal_handle, cobot_move_signal)
			if move_succeeded:
				goal_handle.succeed()
				self.get_logger().info("Pick and place task complete.")
			else:
				goal_handle.abort()
				self.get_logger().error('An error occurred when moving trays...')

			result.task_successful = move_succeeded
			return result


		'''
		Since the model does not detect individual item on the tray, it cannot tell the cobot to pick up a missing item, and
		therefore the cobot should not load a tray unless it is empty,
		in which case the signal is START_TRAY1_LOAD or START_TRAY2_LOAD
		'''
		if (cobot_move_signal == CobotMovement.START_TRAY1_LOAD.value or
  				cobot_move_signal == CobotMovement.START_TRAY2_LOAD.value):
			move_succeeded = self.load_tray(goal_handle, cobot_move_signal)
			if move_succeeded:
				goal_handle.succeed()
				self.get_logger().info("Pick and place task complete.")
			else:
				goal_handle.abort()
				self.get_logger().error('An error occurred when loading trays...')

			result.task_successful = move_succeeded
			return result

		goal_handle.abort()
		result.task_successful = False
		self.get_logger().error('An error occurred when moving or loading trays...')

		return result

	def send_pick_place_request(self, service):
		'''
		Sends a request to one of the three services: Tray service, Case service, and Chip service.

		Args:
			service (rclpy.service.Service): the service to which the request is sent.

		Returns:
			Future: object representing a task to be completed in the future.
		'''

		self.pick_place_request.detect = True
		future = service.call_async(self.pick_place_request)

		return future

	def send_gripper_request(self, output_pin):
		self.get_logger().info("Sending request to gripper service")
		self.gripper_request.pin = output_pin
		future = self.gripper_cli.call_async(self.gripper_request)

		return future

	def get_chip_position(self, retry_limit=1):
		'''
		Gets the chip position by sending a request to the chip service.
		This function blocks until a valid position (1-48) is returned.

		Args:
			retry_limit (int): the retry limit when the first call fails.

		Returns:
			int: case position as an integer between 1 or 17 or -1 if retry limit is exceeded.
		'''
		retries = 0
		chip_position = None
		while retries < retry_limit + 1:
			self.get_logger().info('Waiting for valid chip signal to become available...')
			future_chip = self.send_pick_place_request(self.chip_cli)
			rclpy.spin_until_future_complete(self.subnode, future_chip)
			chip_position = future_chip.result().signal
			retries = retries + 1
			if 1 <= chip_position <=48:
				break

		return chip_position

	def get_case_position(self, retry_limit=1):
		'''
		Gets the tray position by sending a request to the chip service.
		This function blocks until a valid position (1-17) is returned.

		Args:
			retry_limit (int): the retry limit when the first call fails.

		Returns:
			int: case position as an integer between 1 or 17 or -1 if retry limit is exceeded.
		'''
		retries = 0
		case_position = None
		while retries < retry_limit + 1:
			self.get_logger().info('Waiting for valid case signal to become available...')
			future_case = self.send_pick_place_request(self.case_cli)
			rclpy.spin_until_future_complete(self.subnode, future_case)
			case_position = future_case.result().signal
			retries = retries + 1

			if 1 <= case_position <= 17:
				break

		return case_position

	def get_tray_signal(self, retry_limit=1):
		'''
		Sends the request to get a suggested tray move command.
		This function blocks until a non-None signal is received.

		Args:
			retry_limit (int): the retry limit when the first call fails.

		Returns:
			int: tray signal as an integer value of the enum CobotMovement or None if retry limit is exceeded.
		'''

		tray_load_signal = None
		retries = 0
		while retries < retry_limit + 1:
			future_tray = self.send_pick_place_request(self.tray_cli)
			rclpy.spin_until_future_complete(self.subnode, future_tray)
			tray_load_signal = future_tray.result().signal
			self.get_logger().info(f"Tray signal is {CobotMovement(tray_load_signal).name}.")

			if tray_load_signal != None:
				break

			self.get_logger().info("Retrying in 5 seconds")
			time.sleep(5)
			retries = retries + 1

		return tray_load_signal

	def populate_tray_load_trajectories(self, tray_move):
		'''
		Get the trajectories to load items onto the specified tray.

		Args:
			tray_move (CobotMovement): tray load command.

		Returns:
			tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
		'''
		chip_position = self.get_chip_position()
		case_position = self.get_case_position()

		self.get_logger().info(f"Populating tray load trajectories...")
		return get_all_trajectories(
			self.joints,
			self.goals,
			self.gripper_outputs,
			chip_position,
			case_position,
			tray_move)

	def populate_tray_movement_trajectories(self, tray_move):
		'''
		Get the trajectories to move the specified tray to assembly or vice versa.

		Args:
			tray_move (CobotMovement): tray move command.

		Returns:
			tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
		'''
		self.get_logger().info(f"Populating tray movement trajectories...")
		return get_tray_movement_trajectories(
			self.joints,
			self.goals,
			self.gripper_outputs,
			tray_move)

	def load_tray(self, goal_handle, tray_signal):
		self.get_logger().info("Loading tray...")

		feedback_msg = PickPlaceAction.Feedback()
		feedback_msg.current_movement = self.current_movement

		(trajectories, trajectory_names) = self.populate_tray_load_trajectories(tray_signal)

		if not self.starting_point_ok:
			return False

		if trajectories is None or len(trajectories) == 0:
			self.get_logger().error("No trajectories returned")
			return False

		for i, trajectory in enumerate(trajectories):
			# Check if trajectory is a JointTrajectory. i.e. A cobot movement.
			if isinstance(trajectory, JointTrajectory):
				feedback_msg.current_movement = trajectory_names[i]
				self.get_logger().info(f'Goal Name: {feedback_msg.current_movement}')
				goal_handle.publish_feedback(feedback_msg)
				trajectory_goal = trajectory.points[0]

				# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
				pos = f'[Base: {trajectory_goal.positions[0]}, Shoulder: {trajectory_goal.positions[1]}, Elbow: {trajectory_goal.positions[2]}, ' +\
				f'Wrist 1: {trajectory_goal.positions[3]}, Wrist 2: {trajectory_goal.positions[4]}, Wrist 3: {trajectory_goal.positions[5]}]'

				# Using goals as dict type
				self.get_logger().info(f"Sending goal:\n\t{pos}.\n")

				self._publisher.publish(trajectory)

				# Wait for number of seconds defined in config file
				time.sleep(self.wait_sec_between_publish)

			# Otherwise the trajectory is a gripper movement.
			else:
				future_grip = self.send_gripper_request(trajectory)
				rclpy.spin_until_future_complete(self.subnode, future_grip)
				time.sleep(1)

		self.get_logger().info("Tray loaded.")

		return True

	def move_tray(self, goal_handle, tray_signal):
		self.get_logger().info("Moving tray...")

		feedback_msg = PickPlaceAction.Feedback()
		feedback_msg.current_movement = self.current_movement

		(trajectories, trajectory_names) = self.populate_tray_movement_trajectories(tray_signal)

		if not self.starting_point_ok:
			return False

		if trajectories is None:
			self.get_logger().error("No trajectories returned")
			return False

		if len(trajectories) == 0:
			self.get_logger().info("Not moving trays...")
			return True

		for i, trajectory in enumerate(trajectories):
			# Check if trajectory is a JointTrajectory. i.e. A cobot movement.
			if isinstance(trajectory, JointTrajectory):
				feedback_msg.current_movement = trajectory_names[i]
				self.get_logger().info(f'Goal Name: {feedback_msg.current_movement}')
				goal_handle.publish_feedback(feedback_msg)
				trajectory_goal = trajectory.points[0]

				# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
				pos = f'[Base: {trajectory_goal.positions[0]}, Shoulder: {trajectory_goal.positions[1]}, Elbow: {trajectory_goal.positions[2]}, ' +\
				f'Wrist 1: {trajectory_goal.positions[3]}, Wrist 2: {trajectory_goal.positions[4]}, Wrist 3: {trajectory_goal.positions[5]}]'

				# Using goals as dict type
				self.get_logger().info(f"Sending goal:\n\t{pos}.\n")

				self._publisher.publish(trajectory)

				# Wait for number of seconds defined in config file
				time.sleep(self.wait_sec_between_publish)

			# Otherwise the trajectory is a gripper movement.
			else:
				future_grip = self.send_gripper_request(trajectory)
				rclpy.spin_until_future_complete(self.subnode, future_grip)
				time.sleep(1)

		self.get_logger().info("Tray moved.")

		return True

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

				temp.points.append(self.goals["home"])
				temp_traj.append(copy.deepcopy(temp))
				temp.points.clear()

				for traj in temp_traj:
					temp_pos = traj
					self.joint_state_pub.publish(temp_pos)

					time.sleep(self.wait_sec_between_publish)

				self.starting_point_ok = True
			else:
				self.starting_point_ok = True

			self.joint_state_msg_received = True
		else:
			return
