from trajectory_msgs.msg import JointTrajectory
import copy


class BotMethods:
	'''
	BotMethods defines all functions that involve moving the cobot to pick and place each object.
	'''
	# trajectories = {}

	# @staticmethod
	# def get_trajectories():
	# 	# calculate and get trajectories
	# 	pass

	# A list to store joint trajectories.
	trajectories = []
	# A list to store the names of the trajectories.
	trajectory_names = []

	@staticmethod
	def move_home(joints, goals):
		# Create a JointTrajectory object for moving to the home position.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Append the "home" goal to the trajectory.
		traj.points.append(goals["home"])

		# Copy the trajectory and name to the lists.
		BotMethods.trajectories.append(copy.deepcopy(traj))
		BotMethods.trajectory_names.append("home")

		# Clear the trajectory for future use.
		traj.points.clear()

	@staticmethod
	def move_chip(joints, goals, chip_number, tray_number):
		traj = JointTrajectory()
		traj.joint_names = joints
		chip_name = f"chip_{str(chip_number)}"
		above_tray_name = f"above_tray_{str(tray_number)}"
		chip_place_name = f"chip_place_{str(tray_number)}"

		'''
		traj.points must be cleared before appending a new trajectory to it.

		Otherwise, two goals will be within the same trajectory and the trajectory will be invalid.

		Trajectories are added to the trajectories list which is returned so that the cobot will move to all goals.
		'''

		if chip_number < 13:
			chip_home_name = "chip_home_1"
		elif chip_number < 25:
			chip_home_name = "chip_home_2"
		elif chip_number < 37:
			chip_home_name = "chip_home_3"
		else:
			chip_home_name = "chip_home_4"

		# Sequence of goals to move the cobot to
		goal_names = [
			chip_home_name,
			chip_name,
			chip_home_name,
			above_tray_name,
			chip_place_name,
			above_tray_name
		]

		for goal_name in goal_names:
			traj.points.append(goals[goal_name])
			BotMethods.trajectories.append(copy.deepcopy(traj))
			BotMethods.trajectory_names.append(goal_name)
			traj.points.clear()

	@staticmethod
	def move_case(joints, goals, case_number, tray_number):
		# Create a JointTrajectory object for moving to pick and place a case.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Define goal names for the case movement.
		case_name = f"case_{str(case_number)}"
		above_tray_name = f"above_tray_{str(tray_number)}"
		case_place_name = f"bottom_case_place_{str(tray_number)}"
		goal_names = [
			"case_pick_home",
			case_name,
			"case_pick_home",
			above_tray_name,
			case_place_name,
			above_tray_name
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			traj.points.append(goals[goal_name])
			BotMethods.trajectories.append(copy.deepcopy(traj))
			BotMethods.trajectory_names.append(goal_name)
			traj.points.clear()

	@staticmethod
	def move_battery(joints, goals, tray_number):
		# Create a JointTrajectory object for moving to pick and place a battery.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Define goal names for the battery movement.
		above_tray_name = f"above_tray_{str(tray_number)}"
		battery_place_name = f"battery_place_{str(tray_number)}"
		goal_names = [
			"home",
			"battery_pick_home",
			"battery_pick",
			"battery_pick_home",
			"safe_start",
			above_tray_name,
			battery_place_name,
			above_tray_name
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			traj.points.append(goals[goal_name])
			BotMethods.trajectories.append(copy.deepcopy(traj))
			BotMethods.trajectory_names.append(goal_name)
			traj.points.clear()

	@staticmethod
	def move_tray(joints, goals, tray_number):
		# Create a JointTrajectory object for moving to pick and place a tray.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Define goal names for the tray movement.
		above_tray_name = f"above_tray_{str(tray_number)}"
		tray_pick_name = f"tray_pick_{str(tray_number)}"
		goal_names = [
			tray_pick_name,
			above_tray_name,
			"tray_unload_above",
			"tray_unload",
			"tray_unload_above",
			"home",
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			traj.points.append(goals[goal_name])
			BotMethods.trajectories.append(copy.deepcopy(traj))
			BotMethods.trajectory_names.append(goal_name)
			traj.points.clear()

	@staticmethod
	def replace_tray(joints, goals, tray_number):
		# Create a JointTrajectory object for replacing a tray.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Define goal names for the tray replacement.
		above_tray_name = f"above_tray_{str(tray_number)}"
		tray_pick_name = f"tray_pick_{str(tray_number)}"
		goal_names = [
			"tray_unload_above",
			"tray_unload",
			"tray_unload_above",
			above_tray_name,
			tray_pick_name,
			above_tray_name,
			"home",
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			traj.points.append(goals[goal_name])
			BotMethods.trajectories.append(copy.deepcopy(traj))
			BotMethods.trajectory_names.append(goal_name)
			traj.points.clear()

	@staticmethod
	def get_all_trajectories(joints, goals, chip_number, case_number, tray_number):
		# Generate and return a list of all trajectories for various movements.
		BotMethods.move_home(joints, goals)
		BotMethods.move_chip(joints, goals, chip_number, tray_number)
		BotMethods.move_case(joints, goals, case_number, tray_number)
		BotMethods.move_battery(joints, goals, tray_number)
		BotMethods.move_tray(joints, goals, tray_number)
		BotMethods.replace_tray(joints, goals, tray_number)

		return BotMethods.trajectories

	# Must be called after get_all_trajectories
	@staticmethod
	def get_trajectory_names():
		return BotMethods.trajectory_names
