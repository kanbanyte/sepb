from trajectory_msgs.msg import JointTrajectory


class BotMethods:
	"""
	BotMethods defines all functions that involve moving the cobot to pick and place each object.
	"""

	@classmethod
	# def move_chip(self, joints, goals, chip_number):
	# def move_chip(self, pjt, chip_number):
	def move_chip(self, pjt, joints, goals, chip_number):
		trajectories = []
		traj = JointTrajectory()
		traj.joint_names = joints
		# traj.joint_names = pjt.joints
		chip_name = f"chip_{str(chip_number)}"

		# self.get_logger().info(f'chip_name: {chip_name}')
		pjt.get_logger().info(f'chip_name: {chip_name}')

		"""
		traj.points must be cleared before appending a new trajectory to it.

		Otherwise, two goals will be within the same trajectory and the trajectory will be invalid.

		Trajectories are added to the trajectories list which is returned so that the cobot will move to all goals.
		"""
		# traj.points.append(goals["home"])
		# traj.points.append(pjt.goals["home"])
		# trajectories.append(traj)
		# traj.points.clear()

		traj.points.append(goals[chip_name])
		# traj.points.append(pjt.goals[chip_name])
		trajectories.append(traj)
		# print(traj)
		# print(trajectories)
		traj.points.clear()

		traj.points.append(goals["home"])
		# traj.points.append(pjt.goals["home"])
		trajectories.append(traj)
		# print(traj)
		# print(trajectories)
		# traj.points.clear()

		return trajectories
