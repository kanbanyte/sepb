#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.srv import SetCommand


class OnRobotRGNode(Node):
    """Class to handle setting commands."""
    def __init__(self):
        super().__init__('OnRobotRGSimpleControllerServer')
        self.gtype = self.declare_parameter('/onrobot/gripper', 'rg6')

        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.command = OnRobotRGOutput()
        self.set_command_srv = self.create_service(
            SetCommand,
            "/onrobot_rg/set_command",
            self.handle_set_command)

    def handle_set_command(self, req):
        """To handle sending commands via socket connection."""
        self.get_logger().info(str(req.command))
        self.command = self.genCommand(str(req.command), self.command)
        self.pub.publish(self.command)
        #rospy.sleep(1)
        return SetCommand.Response(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def genCommand(self, char, command):
        """Updates the command according to the character entered by the user."""

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

        if char == 'c':
            command.r_gfr = max_force
            command.r_gwd = 0
            command.r_ctr = 16
        elif char == 'o':
            command.r_gfr = max_force
            command.r_gwd = max_width
            command.r_ctr = 16
        elif char == 'i':
            command.r_gfr += 25
            command.r_gfr = min(max_force, command.r_gfr)
            command.r_ctr = 16
        elif char == 'd':
            command.r_gfr -= 25
            command.r_gfr = max(0, command.r_gfr)
            command.r_ctr = 16
        else:
            # If the command entered is a int, assign this value to r_gwd
            try:
                command.r_gfr = max_force
                command.r_gwd = min(max_width, int(char))
                command.r_ctr = 16
            except ValueError:
                pass

        return command

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
