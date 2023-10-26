# Import necessary ROS 2 message types and classes.
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectoryPoint


def read_positions_from_parameters(action_server, goal_names: list):
	# Initialize an empty dictionary to store goal positions.
	goals = {}

	# Iterate through the list of goal names.
	for name in goal_names:
		# Declare a parameter with the current goal name and enable dynamic typing.
		action_server.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

		# Get the value of the parameter, which represents the goal.
		goal = action_server.get_parameter(name).value

		# If the goal is defined as a list (deprecated format), handle it accordingly.
		if isinstance(goal, list):
			# Display a warning message about the deprecated format.
			action_server.get_logger().warn(
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
				action_server.declare_parameter(param_name, [float()])
				param_value = action_server.get_parameter(param_name).value

				float_values = []

				if len(param_value) != len(action_server.joints):
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

			else:
				# Display a warning message for incorrectly defined goals.
				action_server.get_logger().warn(
					f'Goal "{name}" definition is wrong.\nThis goal will not be used.\nUse the following structure:\n'
					"<goal_name>:\n"
					"positions: [joint1, joint2, joint3, ...]\n"
					"velocities: [v_joint1, v_joint2, ...]\n"
					"accelerations: [a_joint1, a_joint2, ...]\n"
					"effort: [eff_joint1, eff_joint2, ...]"
				)

	# Return the dictionary of goal positions.
	return goals

def read_gripper_outputs_from_parameters(action_server, output_names: list):
	'''
	Returns the gripper output values corresponding to the output names.

	Args:
		action_server (CobotMovementActionServer): the Cobot Movement Action Server.
		output_names (list[str]): list containing the names of all possible gripper outputs.

	Returns:
		dict[int]: dictionary containing the gripper output values.
	'''

	outputs = {}
	for name in output_names:
		# Declare a parameter with the current output name and enable dynamic typing.
		action_server.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

		# Get the value of the parameter, which represents the output.
		output = action_server.get_parameter(name).value

		if not isinstance(output, int):
			action_server.get_logger().error(f"Gripper output '{name}' must be of type int. Is type {type(output)}")
			raise TypeError(f"Expected type int, got type {type(output)}")

		one_ok = True

		# If the output is ok, place it in the outputs dictionary using the name as key
		if one_ok:
			outputs[name] = output

		else:
			# Display a warning message for incorrectly defined outputs.
			action_server.get_logger().warn(
				f'Gripper output "{name}" definition is wrong.\nThis output will not be used.\nUse the following structure:\n'
				"<gripper_output_name>: <int_output_value>"
			)

	return outputs
