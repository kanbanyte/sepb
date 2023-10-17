# Import necessary ROS 2 message types and classes.
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectoryPoint


# Define a class for reading methods that affect terminal output.
class ReadMethods:
	'''
	ReadMethods defines all functions that affect the output inside the terminal.
	'''

	@staticmethod
	def read_positions_from_parameters(pjt, goal_names: list):
		# Initialize an empty dictionary to store goal positions.
		goals = {}

		# Iterate through the list of goal names.
		for name in goal_names:
			# Declare a parameter with the current goal name and enable dynamic typing.
			pjt.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

			# Get the value of the parameter, which represents the goal.
			goal = pjt.get_parameter(name).value

			# If the goal is defined as a list (deprecated format), handle it accordingly.
			if isinstance(goal, list):
				# Display a warning message about the deprecated format.
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

				# Convert the values in the list to floats.
				float_goal = [float(value) for value in goal]

				# Create a JointTrajectoryPoint and set its positions and time_from_start.
				point = JointTrajectoryPoint()
				point.positions = float_goal
				point.time_from_start = Duration(sec=4)

				# Store the point in the goals dictionary under the current name.
				goals[name] = point

			# If the goal is not a list, handle it with separate sub-parameters.
			else:
				point = JointTrajectoryPoint()

				# Define a helper function to get sub-parameters like positions, velocities, accelerations, or effort.
				def get_sub_param(sub_param):
					param_name = name + "." + sub_param
					pjt.declare_parameter(param_name, [float()])
					param_value = pjt.get_parameter(param_name).value

					float_values = []

					if len(param_value) != len(pjt.joints):
						return [False, float_values]

					float_values = [float(value) for value in param_value]

					return [True, float_values]

				# Flag to track if at least one sub-parameter is valid.
				one_ok = False

				# Try to get and set positions, velocities, accelerations, and effort sub-parameters.
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

				# If at least one sub-parameter is valid, set time_from_start and store in goals.
				if one_ok:
					point.time_from_start = Duration(sec=4)
					goals[name] = point

					# Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3
					pos = f'[Base: {point.positions[0]}, Shoulder: {point.positions[1]}, Elbow: {point.positions[2]}, ' +\
					f'Wrist 1: {point.positions[3]}, Wrist 2: {point.positions[4]}, Wrist 3: {point.positions[5]}]'

					# Display the goal's positions in the terminal.
					# pjt.get_logger().info(f'\n\tGoal "{name}":\n\t\t{pos}\n')

				else:
					# Display a warning message for incorrectly defined goals.
					pjt.get_logger().warn(
						f'Goal "{name}" definition is wrong.\nThis goal will not be used.\nUse the following structure:\n'
						"<goal_name>:\n"
						"positions: [joint1, joint2, joint3, ...]\n"
						"velocities: [v_joint1, v_joint2, ...]\n"
						"accelerations: [a_joint1, a_joint2, ...]\n"
						"effort: [eff_joint1, eff_joint2, ...]"
					)

		# Return the dictionary of goal positions.
		return goals

	@staticmethod
	def read_gripper_outputs_from_parameters(pjt, output_names: list):
		# Initialize an empty dictionary to store gripper outputs.
		outputs = {}

		# Iterate through the list of output names.
		for name in output_names:
			# Declare a parameter with the current output name and enable dynamic typing.
			pjt.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

			# Get the value of the parameter, which represents the output.
			output = pjt.get_parameter(name).value

			if not isinstance(output, int):
				pjt.get_logger().error(f"Gripper output '{name}' must be of type int. Is type {type(output)}")
				raise TypeError(f"Expected type int, got type {type(output)}")

			one_ok = True

			# If the output is ok, place it in the outputs dictionary using the name as key
			if one_ok:
				outputs[name] = output

			else:
				# Display a warning message for incorrectly defined outputs.
				pjt.get_logger().warn(
					f'Gripper output "{name}" definition is wrong.\nThis output will not be used.\nUse the following structure:\n'
					"<gripper_output_name>: <int_output_value>"
				)

		# Return the dictionary of gripper output values.
		return outputs