#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <iomanip>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>

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
        this->declare_parameter("marker_yaw_offset", 0.0); // Offset to align mocap marker front with robot front

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        path_topic_ = this->get_parameter("path_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        output_tum_file_ = this->get_parameter("output_tum_file").as_string();
        marker_yaw_offset_ = this->get_parameter("marker_yaw_offset").as_double();

        if (!output_tum_file_.empty()) {
            tum_stream_.open(output_tum_file_, std::ios::out);
            if (tum_stream_.is_open()) {
                tum_stream_ << std::fixed << std::setprecision(9);
            }
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        mocap_subscriber_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
            "/rigid_bodies", 10,
            std::bind(&PathTrackerNode::mocapCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PathTrackerNode::odomCallback, this, std::placeholders::_1));
        
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

    void mocapCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
        if (!mocap_received_ && !msg->rigidbodies.empty()) {
            const auto& pose = msg->rigidbodies[0].pose;
            
            tf2::Transform mocap_tf;
            mocap_tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
            mocap_tf.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

            tf2::Transform mocap_to_ros;
            mocap_to_ros.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::Matrix3x3 correct_rot(0, 0, -1, -1, 0, 0, 0, 1, 0);
            tf2::Quaternion q_rot;
            correct_rot.getRotation(q_rot);
            mocap_to_ros.setRotation(q_rot);

            tf2::Transform ros_tf = mocap_to_ros * mocap_tf;
            flattenPose(ros_tf);
            
            // Apply compensational yaw offset (marker physically rotated from base_link)
            tf2::Quaternion marker_correction;
            marker_correction.setRPY(0, 0, marker_yaw_offset_);
            tf2::Transform marker_correction_tf(marker_correction, tf2::Vector3(0,0,0));
            ros_tf = ros_tf * marker_correction_tf;

            mocap_start_ = ros_tf;
            mocap_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Mocap initial global origin captured! Ready. Yaw offset applied: %.2f", marker_yaw_offset_);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        if (!mocap_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for mo-cap setup...");
            return;
        }

        double current_msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        if (is_first_message_){
            bag_start_time_ = current_msg_time;
            is_first_message_ = false;
        }

        if (current_msg_time - bag_start_time_ < 0.5) {
            return; // Wait a bit for SLAM to settle
        }

        try {
            auto tf_msg = tf_buffer_->lookupTransform(frame_id_, robot_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.05));

            tf2::Transform current_tf;
            tf2::fromMsg(tf_msg.transform, current_tf);
            flattenPose(current_tf);

            // Calculate inverse of INITIAL odom, effectively placing the origin at 0,0, then offsetting BY THE EXACT MOCAP INITIAL POS
            if (is_first_odom_pose_) {
                odom_start_inv_ = current_tf.inverse();
                is_first_odom_pose_ = false;
            }

            // from local odom trajectory to global (mocap)             
            tf2::Transform perfectly_aligned_tf = mocap_start_ * odom_start_inv_ * current_tf;


            // RVIZ: from local odom to have the same position as tf's (just for visualization, not for TUM storage)  
            geometry_msgs::msg::PoseStamped path_pose;
            path_pose.header.stamp = msg->header.stamp;
            path_pose.header.frame_id = frame_id_;

            path_pose.pose.position.x = current_tf.getOrigin().x();
            path_pose.pose.position.y = current_tf.getOrigin().y();
            path_pose.pose.position.z = 0.0;
            path_pose.pose.orientation = tf2::toMsg(current_tf.getRotation());

            path_msg_.poses.push_back(path_pose);
            path_msg_.header.stamp = msg->header.stamp; 
            path_publisher_->publish(path_msg_);

            // SAVE TUM (save the physically aligned one!)
            if (tum_stream_.is_open()) {
                double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                tum_stream_ << std::fixed << std::setprecision(9) << time_sec << " "
                            << perfectly_aligned_tf.getOrigin().x() << " "
                            << perfectly_aligned_tf.getOrigin().y() << " "
                            << perfectly_aligned_tf.getOrigin().z() << " "
                            << perfectly_aligned_tf.getRotation().x() << " "
                            << perfectly_aligned_tf.getRotation().y() << " "
                            << perfectly_aligned_tf.getRotation().z() << " "
                            << perfectly_aligned_tf.getRotation().w() << "\n";
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF: %s", ex.what());
        }
    }

private:
    std::string odom_topic_, path_topic_, frame_id_, robot_frame_, output_tum_file_;
    double marker_yaw_offset_ = 0.0;
    bool is_first_message_ = true;
    double bag_start_time_ = 0.0;
    
    bool mocap_received_ = false;
    tf2::Transform mocap_start_;

    bool is_first_odom_pose_ = true;
    tf2::Transform odom_start_inv_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    nav_msgs::msg::Path path_msg_;

    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr mocap_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_; 
    std::ofstream tum_stream_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
