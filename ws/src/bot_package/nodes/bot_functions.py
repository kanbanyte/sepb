from trajectory_msgs.msg import JointTrajectory
import copy


class BotMethods:
	"""
	BotMethods defines all functions that involve moving the cobot to pick and place each object.
	"""
	# trajectories = {}

	# @classmethod
	# def get_trajectories(self):
	# 	# calculate and get trajectories
	# 	pass
	trajectories = []

	@classmethod
	def move_chip(self, joints, goals, chip_number, tray_number):
		traj = JointTrajectory()
		traj.joint_names = joints
		chip_name = f"chip_{str(chip_number)}"
		above_tray_name = f"above_tray_{str(tray_number)}"
		chip_place_name = f"chip_place_{str(tray_number)}"
		# chip_home_name = f"chip_home_2"
		# chip_place_name = f"chip_place_1"
		# above_tray_name = f"above_tray_1"
		# pjt.get_logger().info(f'chip_name: {chip_name}')

		"""
		traj.points must be cleared before appending a new trajectory to it.

		Otherwise, two goals will be within the same trajectory and the trajectory will be invalid.

		Trajectories are added to the trajectories list which is returned so that the cobot will move to all goals.

		home
		chip_home_1
		chip_1
		chip_home_1
		above_tray_1
		chip_place_1
		above_tray_1
		home
		"""
		# traj.points.append(goals["safe_start"])
		# trajectories.append(copy.deepcopy(traj))
		# traj.points.clear()

		traj.points.append(goals["home"])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		if chip_number < 13:
			chip_home_name = "chip_home_1"
		elif chip_number < 25:
			chip_home_name = "chip_home_2"
		elif chip_number < 37:
			chip_home_name = "chip_home_3"
		else:
			chip_home_name = "chip_home_4"

		traj.points.append(goals[chip_home_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[chip_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[chip_home_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[above_tray_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[chip_place_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[above_tray_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals["home"])
		BotMethods.trajectories.append(copy.deepcopy(traj))

	@classmethod
	def move_case(self, joints, goals, case_number, tray_number):
		traj = JointTrajectory()
		traj.joint_names = joints
		case_name = f"case_{str(case_number)}"
		above_tray_name = f"above_tray_{str(tray_number)}"
		case_place_name = f"bottom_case_place_{str(tray_number)}"

		traj.points.append(goals["home"])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals["case_pick_home"])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[case_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals["case_pick_home"])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[above_tray_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[case_place_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[above_tray_name])
		BotMethods.trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals["home"])
		BotMethods.trajectories.append(copy.deepcopy(traj))


	@classmethod
	def get_all_trajectories(self, joints, goals, chip_number, case_number, tray_number):
		BotMethods.move_chip(joints, goals, chip_number, tray_number)
		BotMethods.move_case(joints, goals, case_number, tray_number)

		return BotMethods.trajectories
