from trajectory_msgs.msg import JointTrajectory
import copy


class BotMethods:
	"""
	BotMethods defines all functions that involve moving the cobot to pick and place each object.
	"""

	@classmethod
	def move_chip(self, pjt, joints, goals, chip_number):
		trajectories = []
		traj = JointTrajectory()
		traj.joint_names = joints
		chip_name = f"chip_{str(chip_number)}"
		chip_home_name = f"chip_home_1"
		chip_place_name = f"chip_place_1"
		above_tray_name = f"above_tray_1"
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
		trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[chip_home_name])
		trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[chip_name])
		trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[above_tray_name])
		trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[chip_place_name])
		trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals[above_tray_name])
		trajectories.append(copy.deepcopy(traj))
		traj.points.clear()

		traj.points.append(goals["home"])
		trajectories.append(copy.deepcopy(traj))

		return trajectories
