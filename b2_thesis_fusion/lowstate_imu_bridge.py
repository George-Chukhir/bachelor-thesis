#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from unitree_go.msg import LowState


class LowStateImuBridge(Node):
    def __init__(self) -> None:
        super().__init__("lowstate_imu_bridge")

        self.declare_parameter("input_topic", "/lowstate")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("quat_order", "xyzw")
        self.declare_parameter("orientation_covariance", [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01])
        self.declare_parameter("angular_velocity_covariance", [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02])
        self.declare_parameter("linear_acceleration_covariance", [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1])

        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.quat_order = self.get_parameter("quat_order").get_parameter_value().string_value
        self.orientation_covariance = self._get_covariance("orientation_covariance")
        self.angular_velocity_covariance = self._get_covariance("angular_velocity_covariance")
        self.linear_acceleration_covariance = self._get_covariance("linear_acceleration_covariance")

        self.publisher = self.create_publisher(Imu, self.output_topic, 10)
        self.subscription = self.create_subscription(LowState, self.input_topic, self._on_lowstate, 10)

    def _get_covariance(self, name: str) -> List[float]:
        values = self.get_parameter(name).get_parameter_value().double_array_value
        if len(values) != 9:
            self.get_logger().warn(f"{name} should have 9 values; using -1 covariance.")
            return [-1.0] * 9
        return list(values)

    def _on_lowstate(self, msg: LowState) -> None:
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        quat = list(msg.imu_state.quaternion)
        if len(quat) != 4:
            self.get_logger().warn("imu_state.quaternion should have 4 values; skipping message.")
            return

        if self.quat_order.lower() == "xyzw":
            imu.orientation.x = float(quat[0])
            imu.orientation.y = float(quat[1])
            imu.orientation.z = float(quat[2])
            imu.orientation.w = float(quat[3])
        elif self.quat_order.lower() == "wxyz":
            imu.orientation.w = float(quat[0])
            imu.orientation.x = float(quat[1])
            imu.orientation.y = float(quat[2])
            imu.orientation.z = float(quat[3])
        else:
            self.get_logger().warn("quat_order must be 'xyzw' or 'wxyz'; defaulting to 'xyzw'.")
            imu.orientation.x = float(quat[0])
            imu.orientation.y = float(quat[1])
            imu.orientation.z = float(quat[2])
            imu.orientation.w = float(quat[3])

        gyro = list(msg.imu_state.gyroscope)
        if len(gyro) >= 3:
            imu.angular_velocity.x = float(gyro[0])
            imu.angular_velocity.y = float(gyro[1])
            imu.angular_velocity.z = float(gyro[2])

        accel = list(msg.imu_state.accelerometer)
        if len(accel) >= 3:
            imu.linear_acceleration.x = float(accel[0])
            imu.linear_acceleration.y = float(accel[1])
            imu.linear_acceleration.z = float(accel[2])

        imu.orientation_covariance = self.orientation_covariance
        imu.angular_velocity_covariance = self.angular_velocity_covariance
        imu.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.publisher.publish(imu)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LowStateImuBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
