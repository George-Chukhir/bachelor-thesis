#include <rclcpp/rclcpp.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <fstream>
#include <iomanip>
#include <string>

#include <tf2/utils.h>


/*
    // maybe change topic name
    ros2 run b2_thesis_fusion mocap_to_tum --ros-args -p output_file:=/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/bag5_trajectories/mocap_gt.tum
*/

class MocapToTumNode : public rclcpp::Node
{
public:
    MocapToTumNode() : rclcpp::Node("mocap_to_tum")
    {
        this->declare_parameter("topic_name", "/rigid_bodies");
        this->declare_parameter("output_file", "mocap_gt.tum");

        std::string topic_name = this->get_parameter("topic_name").as_string();
        std::string output_file = this->get_parameter("output_file").as_string();

        file_.open(output_file, std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Listening to %s and writing to %s", topic_name.c_str(), output_file.c_str());
        }

        subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
            topic_name, 10,
            std::bind(&MocapToTumNode::poseCallback, this, std::placeholders::_1));
    }

    ~MocapToTumNode() {
        if (file_.is_open()) file_.close();
    }
    
private:
    std::ofstream file_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
    
    void poseCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
    {
        if (file_.is_open() && !msg->rigidbodies.empty()) {
            const auto& pose = msg->rigidbodies[0].pose;
            double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

            tf2::Transform mocap_tf;
            mocap_tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
            mocap_tf.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

            // Mo-Cap (xz) to ROS (xy) transformation
            // X_ros = -Z_mocap, Y_ros = -X_mocap, Z_ros = Y_mocap
            tf2::Transform mocap_to_ros;
            mocap_to_ros.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::Matrix3x3 correct_rot(0, 0, -1, -1, 0, 0, 0, 1, 0);
            tf2::Quaternion q_rot;
            correct_rot.getRotation(q_rot);
            mocap_to_ros.setRotation(q_rot);

            tf2::Transform ros_tf = mocap_to_ros * mocap_tf;


            // Flatten the pose to 2D (z=0, roll=pitch=0) 
            ros_tf.getOrigin().setZ(0.0);
            double yaw = tf2::getYaw(ros_tf.getRotation());
            tf2::Quaternion q_flat;
            q_flat.setRPY(0.0, 0.0, yaw);
            ros_tf.setRotation(q_flat);

            // storage data to tum
            file_ << std::fixed << std::setprecision(9) << time_sec << " "
             << ros_tf.getOrigin().x() << " "
             << ros_tf.getOrigin().y() << " "
             << 0.0 << " "
             << q_flat.x() << " "
             << q_flat.y() << " "
             << q_flat.z() << " "
             << q_flat.w() << "\n";
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapToTumNode>());
    rclcpp::shutdown();
    return 0;
}
