#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput

class OnRobotRGSimpleController(Node):
    def __init__(self):
        super().__init__('OnRobotRGSimpleController')
        self.gtype = self.declare_parameter('/onrobot/gripper', 'rg6').get_parameter_value().string_value
        self.get_logger().info(self.get_name() + " Using gripper type " + self.gtype)

        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command = OnRobotRGOutput()


    def genCommand(self, char):
        """Updates the command according to the character entered by the user."""

        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            self.get_logger().error(
                self.get_name() + ": Select the gripper type from rg2 or rg6.")
            rclpy.shutdown()

        if char == 'c':
            self.command.r_gfr = 400
            self.command.r_gwd = 0
            self.command.r_ctr = 16
        elif char == 'o':
            self.command.r_gfr = 400
            self.command.r_gwd = max_width
            self.command.r_ctr = 16
        elif char == 'i':
            self.command.r_gfr = min(max_force, self.command.r_gfr + 25)
            self.command.r_ctr = 16
        elif char == 'd':
            self.command.r_gfr = max(0, self.command.r_gfr - 25)
            self.command.r_ctr = 16
        else:
            # If the command entered is a int, assign this value to r_gwd
            try:
                self.command.r_gfr = 400
                self.command.r_gwd = min(max_width, int(char))
                self.command.r_ctr = 16
            except ValueError:
                pass


    def askForCommand(self):
        """Asks the user for a command to send to the gripper."""

        currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
        currentCommand += ' r_gfr = ' + str(self.command.r_gfr)
        currentCommand += ', r_gwd = ' + str(self.command.r_gwd)
        currentCommand += ', r_ctr = ' + str(self.command.r_ctr)

        self.get_logger().info(currentCommand)

        strAskForCommand = '-----\nAvailable commands\n\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += '(0 - max width): Go to that position\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'

        strAskForCommand += '-->'

        return input(strAskForCommand)


    def timer_callback(self):
        """Main loop which requests new commands and
        publish them on the OnRobotRGOutput topic.
        """
        self.genCommand(self.askForCommand())
        self.pub.publish(self.command)



def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGSimpleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

