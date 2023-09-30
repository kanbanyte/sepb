#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGInput


class OnRobotBaseRG(Node):
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot RG gripper.
    """

    def __init__(self, node_name="onrobotbaseRG"):
        # Initiate output message as an empty list
        super().__init__(node_name)

        self.gtype = self.declare_parameter('/onrobot/gripper', 'rg6')
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            self.get_logger().error(
                self.get_name() +
                ": Select the gripper type from rg2 or rg6.")
            rclpy.shutdown()

        command.r_gfr = max(0, command.r_gfr)
        command.r_gfr = min(max_force, command.r_gfr)
        command.r_gwd = max(0, command.r_gwd)
        command.r_gwd = min(max_width, command.r_gwd)

        # Verify that the selected mode number is available
        if command.r_ctr not in [1, 8, 16]:
            self.get_logger(
                self.get_name() +
                ": Select the mode number from" +
                "1 (grip), 8 (stop), or 16 (grip_w_offset).")

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Updates the command which will be sent
            during the next sendCommand() call.
        """

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        self.message.append(command.r_gfr)
        self.message.append(command.r_gwd)
        self.message.append(command.r_ctr)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def restartPowerCycle(self):
        """Restarts the power cycle of the Gripper."""

        self.client.restartPowerCycle()

    def getStatus(self):
        """Requests the status from the gripper and
            return it in the OnRobotRGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotRGInput()

        # Assign the values to their respective variables
        message.g_fof = status[0]
        message.g_gwd = status[9]
        message.g_sta = status[10]
        message.g_wdf = status[17]

        return message
