#!/usr/bin/env python3

import rospy
from onrobot_vg_control.msg import OnRobotVGInput


def printStatus(status):
    """Prints the status string generated by the statusInterpreter function."""

    rospy.loginfo(statusInterpreter(status))


def OnRobotVGStatusListener():
    """Initializes the node and subscribe to the OnRobotVGInput topic."""

    rospy.init_node(
        'OnRobotVGStatusListener', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("OnRobotVGInput", OnRobotVGInput, printStatus)
    rospy.spin()


def statusInterpreter(status):
    """Generates a string according to the current value
       of the status variables.
    """

    output = '\n-----\nOnRobot VG status interpreter\n-----\n'

    # gVCA
    output += 'gVCA = ' + str(status.gVCA) + ': '
    output += 'Current vacuum value on Channel A: ' + str(status.gVCA) + '\n'

    # gVCB
    output += 'gVCB = ' + str(status.gVCB) + ': '
    output += 'Current vacuum value on Channel B: ' + str(status.gVCB) + '\n'

    return output


if __name__ == '__main__':
    OnRobotVGStatusListener()
