from trajectory_msgs.msg import JointTrajectory


class BotMethods:
	"""
	BotMethods defines all functions that involve moving the cobot to pick and place each object.
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

		Trajectories are added to the trajectories list which is returned so that the cobot will move to all goals.
		"""
		traj.points.append(goals["home"])
		trajectories.append(traj)
		traj.points.clear()

		traj.points.append(goals[chip_name])
		trajectories.append(traj)
		traj.points.clear()

		traj.points.append(goals["home"])
		trajectories.append(traj)

		return trajectories
