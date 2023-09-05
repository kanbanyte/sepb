#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node {
private:
    size_t _count;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _broadcaster;

    double _degree;
    rclcpp::Rate _loop_rate;

    // Robot state variables
    double _tilt;
    double _tinc;
    double _swivel;
    double _angle;
    double _height;
    double _hinc;

    // Message declarations
    geometry_msgs::msg::TransformStamped _odom_trans;
    sensor_msgs::msg::JointState _joint_state;

    // Function to convert Euler angles to a Quaternion
    geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion q;
        q.x = quat.x();
        q.y = quat.y();
        q.z = quat.z();
        q.w = quat.w();
        return q;
    }

public:
    StatePublisher() : Node("state_publisher"), _count(0) {
        // Define Quality of Service(QoS) profile for communication
        rclcpp::QoS qos_profile(10);

        // Create a publisher for joint states
        _joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", qos_profile);

        // Create a TransformBroadcaster for sending transforms
        _broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Constants and loop rate
        _degree = M_PI / 180.0;
        _loop_rate = rclcpp::Rate(30);

        // Initialize robot state variables
        _tilt = 0.0;
        _tinc = _degree;
        _swivel = 0.0;
        _angle = 0.0;
        _height = 0.0;
        _hinc = 0.005;

        // Message declarations
        _odom_trans.header.frame_id = "odom";
        _odom_trans.child_frame_id = "axis";
        _joint_state.header.frame_id = "axis";
        _joint_state.name = {"swivel", "tilt", "periscope"};

        try {
            while(rclcpp::ok()) {
                // rclcpp::spin(*this);
                // rclcpp::Executor::spin_node_once(this);
                rclcpp::spin_some(Node);

                // Update joint_state
                rclcpp::Time now = get_clock()->now();
                _joint_state.header.stamp = now;
                _joint_state.position = {_swivel, _tilt, _height};

                // Update transform
                // (moving in a circle with radius=2)
                _odom_trans.header.stamp = now;
                _odom_trans.transform.translation.x = cos(_angle) * 2.0;
                _odom_trans.transform.translation.y = sin(_angle) * 2.0;
                _odom_trans.transform.translation.z = 0.7;
                _odom_trans.transform.rotation = euler_to_quaternion(0, 0, _angle + M_PI / 2); // roll, pitch, yaw

                // Publish the joint state and transform
                _joint_pub->publish(_joint_state);
                _broadcaster->sendTransform(_odom_trans);

                // Create new robot state
                _tilt += _tinc;
                if(_tilt < -0.5 || _tilt > 0.0) _tinc *= -1;
                _height += _hinc;
                if(_height > 0.2 || _height < 0.0) _hinc *= -1;
                _swivel += _degree;
                _angle += _degree / 4;

                // Sleep to achieve the desired loop rate
                _loop_rate.sleep();
            }
        }
        catch(const std::exception &) {
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}
