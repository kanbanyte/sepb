from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class BotMethods:
	"""
	BotMethods defines all functions that
	"""

	@classmethod
	def move_chip(self, joints, goals, chip_number):
		trajectories = []

		traj = JointTrajectory()
		traj.joint_names = joints

		chip_name = "chip_" + str(chip_number)


		"""
		traj.points must be cleared before appending a new trajectory to it.

		Otherwise, two goals will be within the same trajectory and the trajectory will be invalid.
		"""
		traj.points.append(goals["home"])
		trajectories.append(traj)
		traj.points.clear()
		traj.points.append(goals[chip_name])

		return traj
