from bot_move import PublisherJointTrajectory
import rclpy


def main(args=None):
	rclpy.init(args=args)

	publisher_joint_trajectory = PublisherJointTrajectory()

	rclpy.spin(publisher_joint_trajectory)
	publisher_joint_trajectory.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
