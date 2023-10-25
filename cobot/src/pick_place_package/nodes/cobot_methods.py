from trajectory_msgs.msg import JointTrajectory
import copy
from data_processing.tray_position import CobotMovement

def __get_home_trajectories(joints, goals, gripper_outputs):
	'''
	Get trajectories to move back to the Home position

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''
	trajectories = []
	trajectory_names = []
	# Create a JointTrajectory object for moving to the home position.
	traj = JointTrajectory()
	traj.joint_names = joints

	# Append the "home" goal to the trajectory.
	traj.points.append(goals["home"])

	# Copy the trajectory and name to the lists.
	trajectories.append(copy.deepcopy(traj))
	trajectory_names.append("home")

	trajectories.append(gripper_outputs["gripper_open_home"])
	trajectory_names.append("gripper_open_home")

	# Clear the trajectory for future use.
	traj.points.clear()

	return (trajectories, trajectory_names)

def __get_chip_move_trajectories(joints, goals, gripper_outputs, chip_number, tray_number):
	'''
	Get trajectories to move the chip to the tray specified by `tray_number`

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''
	trajectories = []
	trajectory_names = []

	traj = JointTrajectory()
	traj.joint_names = joints
	chip_name = f"chip_{str(chip_number)}"
	chip_place_above_name = f"chip_place_above_{str(tray_number)}"
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
		chip_place_above_name,
		chip_place_name,
		"gripper_open_case",
		chip_place_above_name
	]

	for goal_name in goal_names:
		if goal_name.startswith("gripper"):
			trajectories.append(gripper_outputs[goal_name])
			trajectory_names.append(goal_name)
		else:
			traj.points.append(goals[goal_name])
			trajectories.append(copy.deepcopy(traj))
			trajectory_names.append(goal_name)
			traj.points.clear()

	return (trajectories, trajectory_names)

def __get_case_move_trajectories(joints, goals, gripper_outputs, case_number, tray_number):
	'''
	Get trajectories to move the case to the tray specified by `tray_number`

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''
	trajectories = []
	trajectory_names = []

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
			trajectories.append(gripper_outputs[goal_name])
			trajectory_names.append(goal_name)
		else:
			traj.points.append(goals[goal_name])
			trajectories.append(copy.deepcopy(traj))
			trajectory_names.append(goal_name)
			traj.points.clear()

	return (trajectories, trajectory_names)

def __get_battery_move_trajectories(joints, goals, gripper_outputs, tray_number):
	'''
	Get trajectories to move the battery to the tray specified by `tray_number`

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''
	trajectories = []
	trajectory_names = []

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
		battery_place_name,
		"gripper_open_battery",
		above_tray_name,
		"home"
	]

	# Append each goal to the trajectory, copy to lists, and clear the trajectory.
	for goal_name in goal_names:
		if goal_name.startswith("gripper"):
			trajectories.append(gripper_outputs[goal_name])
			trajectory_names.append(goal_name)
		else:
			traj.points.append(goals[goal_name])
			trajectories.append(copy.deepcopy(traj))
			trajectory_names.append(goal_name)
			traj.points.clear()

	return (trajectories, trajectory_names)

def __get_tray_move_trajectories(joints, goals, gripper_outputs, tray_number):
	'''
	Get the trajectories used to move the tray in the position specified by `tray_number` to assembly

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''

	trajectories = []
	trajectory_names = []

	# Create a JointTrajectory object for moving to pick and place a tray.
	traj = JointTrajectory()
	traj.joint_names = joints

	# Define goal names for the tray movement.
	above_tray_name = f"tray_pick_{str(tray_number)}_above"
	tray_pick_name = f"tray_pick_{str(tray_number)}"
	goal_names = [
		above_tray_name,
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
			trajectories.append(gripper_outputs[goal_name])
			trajectory_names.append(goal_name)
		else:
			traj.points.append(goals[goal_name])
			trajectories.append(copy.deepcopy(traj))
			trajectory_names.append(goal_name)
			traj.points.clear()

	return (trajectories, trajectory_names)

def __get_tray_replacement_trajectories(joints, goals, gripper_outputs, tray_number):
	'''
	Get the trajectories used to move the assembly tray back to the position specified by `tray_number`

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''

	trajectories = []
	trajectory_names = []

	# Create a JointTrajectory object for replacing a tray.
	traj = JointTrajectory()
	traj.joint_names = joints

	# Define goal names for the tray replacement.
	above_tray_name = f"tray_pick_{str(tray_number)}_above"
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
		"home",
	]

	# Append each goal to the trajectory, copy to lists, and clear the trajectory.
	for goal_name in goal_names:
		if goal_name.startswith("gripper"):
			trajectories.append(gripper_outputs[goal_name])
			trajectory_names.append(goal_name)
		else:
			traj.points.append(goals[goal_name])
			trajectories.append(copy.deepcopy(traj))
			trajectory_names.append(goal_name)
			traj.points.clear()

	return (trajectories, trajectory_names)

def get_tray_movement_trajectories(joints, goals, gripper_outputs, cobot_movement):
	'''
	Get trajectories to move trays to and from the human operator.

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''
	if cobot_movement == CobotMovement.ASSEMBLY_TO_TRAY1.value:
		tray_number = 1
		move_home_trajectories = __get_home_trajectories(joints, goals, gripper_outputs)
		replace_tray_trajectories = __get_tray_replacement_trajectories(joints, goals, gripper_outputs, tray_number)

		return (
			move_home_trajectories[0] + replace_tray_trajectories[0],
			move_home_trajectories[1] + replace_tray_trajectories[1])

	if cobot_movement == CobotMovement.ASSEMBLY_TO_TRAY2.value:
		tray_number = 2
		move_home_trajectories = __get_home_trajectories(joints, goals, gripper_outputs)
		replace_tray_trajectories = __get_tray_replacement_trajectories(joints, goals, gripper_outputs, tray_number)

		return (
			move_home_trajectories[0] + replace_tray_trajectories[0],
			move_home_trajectories[1] + replace_tray_trajectories[1])

	if cobot_movement == CobotMovement.TRAY1_TO_ASSEMBLY.value:
		tray_number = 1
		move_home_trajectories = __get_home_trajectories(joints, goals, gripper_outputs)
		replace_tray_trajectories = __get_tray_move_trajectories(joints, goals, gripper_outputs, tray_number)

		return (
			move_home_trajectories[0] + replace_tray_trajectories[0],
			move_home_trajectories[1] + replace_tray_trajectories[1])

	if cobot_movement == CobotMovement.TRAY2_TO_ASSEMBLY.value:
		tray_number = 2
		move_home_trajectories = __get_home_trajectories(joints, goals, gripper_outputs)
		replace_tray_trajectories = __get_tray_move_trajectories(joints, goals, gripper_outputs, tray_number)

		return (
			move_home_trajectories[0] + replace_tray_trajectories[0],
			move_home_trajectories[1] + replace_tray_trajectories[1])

	return ([], [])

def get_all_trajectories(joints, goals, gripper_outputs, chip_number, case_number, cobot_movement):
	'''
	Get trajectories to load items onto an empty tray.

	Return:
		tuple(list,list): tuple containing 2 lists. The first list contains trajectories, the second contains their names
	'''

	tray_number = None
	if cobot_movement == CobotMovement.START_TRAY1_LOAD.value:
		tray_number = 1
	elif cobot_movement == CobotMovement.START_TRAY2_LOAD.value:
		tray_number = 2
	else:
		return ([], [])

	move_home_trajectories = __get_home_trajectories(joints, goals, gripper_outputs)
	move_chip_trajectories = __get_chip_move_trajectories(joints, goals, gripper_outputs, chip_number, tray_number)
	move_case_trajectories = __get_case_move_trajectories(joints, goals, gripper_outputs, case_number, tray_number)
	move_battery_trajectories = __get_battery_move_trajectories(joints, goals, gripper_outputs, tray_number)

	return (
		move_home_trajectories[0] + move_chip_trajectories[0] + move_case_trajectories[0] + move_battery_trajectories[0],
		move_home_trajectories[1] + move_chip_trajectories[1] + move_case_trajectories[1] + move_battery_trajectories[1]
	)
