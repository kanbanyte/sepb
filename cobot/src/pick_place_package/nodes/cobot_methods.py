from trajectory_msgs.msg import JointTrajectory
import copy
from data_processing.tray_position import TrayMovement, CobotMovement


class CobotMethods:
	'''
	CobotMethods defines all functions that involve moving the cobot to pick and place each object.
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
	def move_home(joints, goals, gripper_outputs):
		# Create a JointTrajectory object for moving to the home position.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Append the "home" goal to the trajectory.
		traj.points.append(goals["home"])

		# Copy the trajectory and name to the lists.
		CobotMethods.trajectories.append(copy.deepcopy(traj))
		CobotMethods.trajectory_names.append("home")

		CobotMethods.trajectories.append(gripper_outputs["gripper_open_home"])
		CobotMethods.trajectory_names.append("gripper_open_home")

		# Clear the trajectory for future use.
		traj.points.clear()

	@staticmethod
	def move_chip(joints, goals, gripper_outputs, chip_number, tray_number):
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
			"gripper_close_chip",
			chip_home_name,
			above_tray_name,
			chip_place_name,
			"gripper_open_case",
			above_tray_name
		]

		for goal_name in goal_names:
			if goal_name.startswith("gripper"):
				CobotMethods.trajectories.append(gripper_outputs[goal_name])
				CobotMethods.trajectory_names.append(goal_name)
			else:
				traj.points.append(goals[goal_name])
				CobotMethods.trajectories.append(copy.deepcopy(traj))
				CobotMethods.trajectory_names.append(goal_name)
				traj.points.clear()

	@staticmethod
	def move_case(joints, goals, gripper_outputs, case_number, tray_number):
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
			"gripper_close_case",
			"case_pick_home",
			above_tray_name,
			case_place_name,
			"gripper_open_battery",
			above_tray_name
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			if goal_name.startswith("gripper"):
				CobotMethods.trajectories.append(gripper_outputs[goal_name])
				CobotMethods.trajectory_names.append(goal_name)
			else:
				traj.points.append(goals[goal_name])
				CobotMethods.trajectories.append(copy.deepcopy(traj))
				CobotMethods.trajectory_names.append(goal_name)
				traj.points.clear()

	@staticmethod
	def move_battery(joints, goals, gripper_outputs, tray_number):
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
			"gripper_close_battery",
			"battery_pick_home",
			"safe_start",
			above_tray_name,
			battery_place_name,
			"gripper_open_battery",
			above_tray_name
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			if goal_name.startswith("gripper"):
				CobotMethods.trajectories.append(gripper_outputs[goal_name])
				CobotMethods.trajectory_names.append(goal_name)
			else:
				traj.points.append(goals[goal_name])
				CobotMethods.trajectories.append(copy.deepcopy(traj))
				CobotMethods.trajectory_names.append(goal_name)
				traj.points.clear()

	@staticmethod
	def move_tray(joints, goals, gripper_outputs, tray_number):
		# Create a JointTrajectory object for moving to pick and place a tray.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Define goal names for the tray movement.
		above_tray_name = f"above_tray_{str(tray_number)}"
		tray_pick_name = f"tray_pick_{str(tray_number)}"
		goal_names = [
			tray_pick_name,
			"gripper_close_tray",
			above_tray_name,
			"tray_unload_above",
			"tray_unload",
			"gripper_open_tray",
			"tray_unload_above",
			"home",
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			if goal_name.startswith("gripper"):
				CobotMethods.trajectories.append(gripper_outputs[goal_name])
				CobotMethods.trajectory_names.append(goal_name)
			else:
				traj.points.append(goals[goal_name])
				CobotMethods.trajectories.append(copy.deepcopy(traj))
				CobotMethods.trajectory_names.append(goal_name)
				traj.points.clear()

	@staticmethod
	def replace_tray(joints, goals, gripper_outputs, tray_number):
		# Create a JointTrajectory object for replacing a tray.
		traj = JointTrajectory()
		traj.joint_names = joints

		# Define goal names for the tray replacement.
		above_tray_name = f"above_tray_{str(tray_number)}"
		tray_pick_name = f"tray_pick_{str(tray_number)}"
		goal_names = [
			"tray_unload_above",
			"tray_unload",
			"gripper_close_tray",
			"tray_unload_above",
			above_tray_name,
			tray_pick_name,
			"gripper_open_tray",
			above_tray_name,
		]

		# Append each goal to the trajectory, copy to lists, and clear the trajectory.
		for goal_name in goal_names:
			if goal_name.startswith("gripper"):
				CobotMethods.trajectories.append(gripper_outputs[goal_name])
				CobotMethods.trajectory_names.append(goal_name)
			else:
				traj.points.append(goals[goal_name])
				CobotMethods.trajectories.append(copy.deepcopy(traj))
				CobotMethods.trajectory_names.append(goal_name)
				traj.points.clear()

	@staticmethod
	def get_all_trajectories(joints, goals, gripper_outputs, chip_number, case_number, tray_movement):
		# Generate and return a list of all trajectories for various movements.
		tray_number = 1
		# if tray_movement == TrayMovement.ASSEMBLY_TO_TRAY1.value:
		# 	tray_number = 1
		# 	CobotMethods.move_home(joints, goals, gripper_outputs)
		# 	CobotMethods.replace_tray(joints, goals, gripper_outputs, tray_number)
		# elif tray_movement == TrayMovement.ASSEMBLY_TO_TRAY2.value:
		# 	tray_number = 2
		# 	CobotMethods.move_home(joints, goals, gripper_outputs)
		# 	CobotMethods.replace_tray(joints, goals, gripper_outputs, tray_number)
		# elif tray_movement == TrayMovement.TRAY1_TO_ASSEMBLY.value:
		# 	CobotMethods.move_home(joints, goals, gripper_outputs)
		# 	CobotMethods.move_tray(joints, goals, gripper_outputs, 1)
		# elif tray_movement == TrayMovement.TRAY2_TO_ASSEMBLY.value:
		# 	CobotMethods.move_home(joints, goals, gripper_outputs)
		# 	CobotMethods.move_tray(joints, goals, gripper_outputs, 2)
		# elif tray_movement == TrayMovement.both_empty.value:
		# 	tray_number = 1
		# elif tray_movement == TrayMovement.NONE.value:
		# 	return None

		if tray_movement == CobotMovement.ASSEMBLY_TO_TRAY1.value:
			tray_number = 1
			CobotMethods.move_home(joints, goals, gripper_outputs)
			CobotMethods.replace_tray(joints, goals, gripper_outputs, tray_number)
		elif tray_movement == CobotMovement.ASSEMBLY_TO_TRAY2.value:
			tray_number = 2
			CobotMethods.move_home(joints, goals, gripper_outputs)
			CobotMethods.replace_tray(joints, goals, gripper_outputs, tray_number)
		elif tray_movement == CobotMovement.TRAY1_TO_ASSEMBLY.value:
			CobotMethods.move_home(joints, goals, gripper_outputs)
			CobotMethods.move_tray(joints, goals, gripper_outputs, 1)
		elif tray_movement == CobotMovement.TRAY2_TO_ASSEMBLY.value:
			CobotMethods.move_home(joints, goals, gripper_outputs)
			CobotMethods.move_tray(joints, goals, gripper_outputs, 2)
		elif tray_movement == CobotMovement.START_TRAY1_LOAD.value:
			tray_number = 1
		elif tray_movement == CobotMovement.START_TRAY2_LOAD.value:
			tray_number = 2
		elif tray_movement == CobotMovement.NONE.value:
			return None

		CobotMethods.move_home(joints, goals, gripper_outputs)
		CobotMethods.move_chip(joints, goals, gripper_outputs, chip_number, tray_number)
		CobotMethods.move_case(joints, goals, gripper_outputs, case_number, tray_number)
		CobotMethods.move_battery(joints, goals, gripper_outputs, tray_number)

		CobotMethods.move_tray(joints, goals, gripper_outputs, tray_number)

		return CobotMethods.trajectories

	# Must be called after get_all_trajectories
	@staticmethod
	def get_trajectory_names():
		return CobotMethods.trajectory_names
