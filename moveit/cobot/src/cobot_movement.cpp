#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
	// Initialize ROS and create the Node
	rclcpp::init(argc, argv);
	auto const node = std::make_shared<rclcpp::Node>(
		"hello_moveit",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
	);
	
	// Create a ROS logger
	auto const logger = rclcpp::get_logger("hello_moveit");
	
	// Create the MoveIt MoveGroup Interface
	using moveit::planning_interface::MoveGroupInterface;

  /*
  Output after running code: 

  [ERROR] [1693128123.977515507] [hello_moveit]: Could not find parameter robot_description_semantic and
  did not receive robot_description_semantic via std_msgs::msg::String subscription within 10.000000 seconds.
  Error:   Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT ErrorID=13 (0xd) Line number=0 at line 715 in ./src/model.cpp
  [ERROR] [1693128124.031428604] [moveit_rdf_loader.rdf_loader]: Unable to parse SRDF
  [FATAL] [1693128124.031690928] [move_group_interface]: Unable to construct robot model. Please make sure all needed information is on the parameter server.
  terminate called after throwing an instance of 'std::runtime_error'
  what():  Unable to construct robot model. Please make sure all needed information is on the parameter server.

  Need to find a way to load the robot description parameter to the node.
  */
	MoveGroupInterface::Options move_group_options("ur_manipulator", "robot_description");
	auto move_group_interface = MoveGroupInterface(node, move_group_options);
  

	// Set a target Pose
	auto const target_pose = []
	{
		geometry_msgs::msg::Pose msg;
		msg.orientation.w = 1.0;
		msg.position.x = 0.28;
		msg.position.y = -0.2;
		msg.position.z = 0.5;
		return msg;
	}();
	move_group_interface.setPoseTarget(target_pose);

	// Create a plan to that target pose
	auto const [success, plan] = [&move_group_interface]
	{
		moveit::planning_interface::MoveGroupInterface::Plan msg;
		auto const ok = static_cast<bool>(move_group_interface.plan(msg));
		return std::make_pair(ok, msg);
	}();

	// Execute the plan
	if(success)
	{
		move_group_interface.execute(plan);
	} 
	else
	{
		RCLCPP_ERROR(logger, "Planning failed!");
	}
	
	// Shutdown ROS
	rclcpp::shutdown();
	return 0;
}
