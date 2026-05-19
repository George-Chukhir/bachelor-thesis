#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <iomanip>
#include <string>


/*
    // maybe change topic name
    ros2 run b2_thesis_fusion mocap_to_tum --ros-args -p topic_name:=/mocap/rigid_bodies/robot/pose -p output_file:=/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/mocap_gt.tum

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

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
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
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (file_.is_open()) {
            double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            file_ << std::fixed << std::setprecision(9) << time_sec << " "
                  << msg->pose.position.x << " "
                  << msg->pose.position.y << " "
                  << msg->pose.position.z << " "
                  << msg->pose.orientation.x << " "
                  << msg->pose.orientation.y << " "
                  << msg->pose.orientation.z << " "
                  << msg->pose.orientation.w << "\n";
        }
    }

    std::ofstream file_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapToTumNode>());
    rclcpp::shutdown();
    return 0;
}
