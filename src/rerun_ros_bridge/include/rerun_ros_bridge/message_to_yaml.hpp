// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#ifndef RERUN_ROS_BRIDGE__MESSAGE_TO_YAML_HPP_
#define RERUN_ROS_BRIDGE__MESSAGE_TO_YAML_HPP_

#include <memory>
#include <string>

#include <rclcpp/serialized_message.hpp>

namespace rerun_ros_bridge
{

/// Renders serialized messages of one type as YAML-like text, using the type's
/// introspection typesupport found at runtime. This lets the bridge log any
/// message type (including application-specific ones it was not compiled
/// against) as text.
class MessageToYaml
{
public:
  /// Loads typesupport for `type` (e.g. "std_msgs/msg/String").
  /// Throws std::runtime_error if the type's libraries cannot be found.
  explicit MessageToYaml(const std::string & type);
  ~MessageToYaml();

  MessageToYaml(const MessageToYaml &) = delete;
  MessageToYaml & operator=(const MessageToYaml &) = delete;

  /// Deserialize and render. Arrays longer than `max_array_items` are truncated.
  std::string render(const rclcpp::SerializedMessage & msg, size_t max_array_items = 32) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rerun_ros_bridge

#endif  // RERUN_ROS_BRIDGE__MESSAGE_TO_YAML_HPP_
