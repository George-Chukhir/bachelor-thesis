#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <iomanip>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class PathTrackerNode : public rclcpp::Node
{
public:
    PathTrackerNode() : rclcpp::Node("path_tracker")
    {
        this->declare_parameter("odom_topic", "odometry/filtered");
        this->declare_parameter("path_topic", "trajectory/path");
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("robot_frame", "base_link");

        this->declare_parameter("output_tum_file", "");
        this->declare_parameter("pose_topic", "");
        
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        path_topic_ = this->get_parameter("path_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();

        
        
        output_tum_file_ = this->get_parameter("output_tum_file").as_string();
        pose_topic_ = this->get_parameter("pose_topic").as_string();

        if (!output_tum_file_.empty()) {
            tum_stream_.open(output_tum_file_, std::ios::out);
            if (tum_stream_.is_open()) {
                tum_stream_ << std::fixed << std::setprecision(9);
            }
        }


        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PathTrackerNode::odomCallback, 
            this, std::placeholders::_1));
        
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
                   
        path_msg_ = nav_msgs::msg::Path();
        path_msg_.header.frame_id = frame_id_;
    }

    ~PathTrackerNode() {
        if (tum_stream_.is_open()) {
            tum_stream_.close();
        }
    }

    void flattenPose(tf2::Transform& t) {
        t.getOrigin().setZ(0.0);
        double roll, pitch, yaw;
        tf2::Matrix3x3(t.getRotation()).getRPY(roll, pitch, yaw);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        t.setRotation(q);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        try {
            // Every time we get odom->base_link, we ask for global position of robot (relative to "map") 
            // frame_id_ will be "odom" for L1-L3, and "map" для L4 (launch params)
            auto tf_msg = tf_buffer_->lookupTransform(frame_id_, robot_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.05));

            geometry_msgs::msg::PoseStamped path_pose;
            path_pose.header.stamp = msg->header.stamp;
            path_pose.header.frame_id = frame_id_;


            path_pose.pose.position.x = tf_msg.transform.translation.x;
            path_pose.pose.position.y = tf_msg.transform.translation.y;
            path_pose.pose.position.z = tf_msg.transform.translation.z;
            path_pose.pose.orientation = tf_msg.transform.rotation;

            // Draw path in RViz
            path_msg_.poses.push_back(path_pose);
            path_msg_.header.stamp = msg->header.stamp; 
            path_publisher_->publish(path_msg_);

            // 2. Save the TUM for evo
            if (tum_stream_.is_open()) {
                double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                tum_stream_ << std::fixed << std::setprecision(9) << time_sec << " "
                            << path_pose.pose.position.x << " "
                            << path_pose.pose.position.y << " "
                            << path_pose.pose.position.z << " "
                            << path_pose.pose.orientation.x << " "
                            << path_pose.pose.orientation.y << " "
                            << path_pose.pose.orientation.z << " "
                            << path_pose.pose.orientation.w << "\n";
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF alignment: %s", ex.what());
        }
    }


private:
    std::string odom_topic_, path_topic_, frame_id_, robot_frame_;
    bool initialized_ = false;
    tf2::Transform t_align_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    nav_msgs::msg::Path path_msg_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_; 



    std::string output_tum_file_;
    std::string pose_topic_;
    std::ofstream tum_stream_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
