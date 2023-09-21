from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectoryPoint


class ReadMethods:
	"""
	ReadMethods defines all functions that affect the output inside the terminal.
	"""

	@classmethod
	def read_positions_from_parameters(self, pjt, goal_names: list):
		# Temp dict of JointTrajectoryPoint
		goals = {}

		for name in goal_names:
			pjt.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))
			goal = pjt.get_parameter(name).value

			if isinstance(goal, list):
				pjt.get_logger().warn(
					f'Goal "{name}" is defined as a list.\nThis is deprecated.\nUse the following structure:\n'
					"<goal_name>:\n"
					"positions: [joint1, joint2, joint3, ...]\n"
					"velocities: [v_joint1, v_joint2, ...]\n"
					"accelerations: [a_joint1, a_joint2, ...]\n"
					"effort: [eff_joint1, eff_joint2, ...]"
				)

				if goal is None or len(goal) == 0:
					raise Exception(f'Values for goal "{name}" not set!')

				float_goal = [float(value) for value in goal]

				point = JointTrajectoryPoint()
				point.positions = float_goal
				point.time_from_start = Duration(sec=4)

				goal[name] = point

			else:
				point = JointTrajectoryPoint()

				def get_sub_param(sub_param):
					param_name = name + "." + sub_param
					pjt.declare_parameter(param_name, [float()])
					param_value = pjt.get_parameter(param_name).value

					float_values = []

					if len(param_value) != len(pjt.joints):
						return [False, float_values]

					float_values = [float(value) for value in param_value]

					return [True, float_values]

				one_ok = False

				[ok, values] = get_sub_param("positions")
				if ok:
					point.positions = values
					one_ok = True

				[ok, values] = get_sub_param("velocities")
				if ok:
					point.velocities = values
					one_ok = True

				[ok, values] = get_sub_param("accelerations")
				if ok:
					point.accelerations = values
					one_ok = True

				[ok, values] = get_sub_param("effort")
				if ok:
					point.effort = values
					one_ok = True

				if one_ok:
					point.time_from_start = Duration(sec=4)
					goals[name] = point

					# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
					pos = f'[Base: {point.positions[0]}, Shoulder: {point.positions[1]}, Elbow: {point.positions[2]}, ' +\
					f'Wrist 1: {point.positions[3]}, Wrist 2: {point.positions[4]}, Wrist 3: {point.positions[5]}]'
					pjt.get_logger().info(f'\n\tGoal "{name}":\n\t\t{pos}\n')

				else:
					pjt.get_logger().warn(
						f'Goal "{name}" definition is wrong.\nThis goal will not be used.\nUse the following structure:\n'
						"<goal_name>:\n"
						"positions: [joint1, joint2, joint3, ...]\n"
						"velocities: [v_joint1, v_joint2, ...]\n"
						"accelerations: [a_joint1, a_joint2, ...]\n"
						"effort: [eff_joint1, eff_joint2, ...]"
					)
		return goals
