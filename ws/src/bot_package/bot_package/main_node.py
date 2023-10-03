import rclpy
# Import the MainActionClient class from the 'main_action_client' module.
from nodes.main_action_client import MainActionClient

def main(args=None):
    rclpy.init(args=args)

    action_client = MainActionClient()

    future = action_client.send_goal(True)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
