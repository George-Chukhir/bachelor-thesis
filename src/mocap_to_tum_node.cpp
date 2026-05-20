#include <rclcpp/rclcpp.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <fstream>
#include <iomanip>
#include <string>


/*
    // maybe change topic name
    ros2 run b2_thesis_fusion mocap_to_tum --ros-args -p topic_name:=/rigid_bodies -p output_file:=/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/mocap_gt.tum

*/

class MocapToTumNode : public rclcpp::Node
{
public:
    MocapToTumNode() : rclcpp::Node("mocap_to_tum")
    {
        this->declare_parameter("topic_name", "/mocap/rigid_bodies/robot/pose");
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

    ~MocapToTumNode()
    {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void poseCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
    {
        if (file_.is_open() && !msg->rigidbodies.empty()) {
            const auto& pose = msg->rigidbodies[0].pose;
            double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            file_ << std::fixed << std::setprecision(9) << time_sec << " "
             << pose.position.x << " "
             << pose.position.z << " "  // ROS Y <- Mocap Z
             << pose.position.y << " "  // ROS Z <- Mocap Y (Высота)
             << pose.orientation.x << " "
             << pose.orientation.z << " " // Кватернионы тоже лучше свапнуть для консистентности
             << pose.orientation.y << " "
             << pose.orientation.w << "\n";
        }
    }

    std::ofstream file_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapToTumNode>());
    rclcpp::shutdown();
    return 0;
}
