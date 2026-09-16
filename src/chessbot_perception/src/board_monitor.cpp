// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

// Perception: camera frames in, board-level facts out, on request.
//
// Holds the latest overhead frame and answers chessbot_interfaces/GetBoardState.
// Board detection itself is not implemented yet (there is no board in the sim);
// the contract, frame bookkeeping and freshness handling are.

#include <memory>
#include <mutex>
#include <string>

#include <chessbot_interfaces/msg/board_state.hpp>
#include <chessbot_interfaces/srv/get_board_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "chessbot_perception/board_detector.hpp"

namespace chessbot_perception
{

using chessbot_interfaces::msg::BoardState;
using chessbot_interfaces::srv::GetBoardState;

class BoardMonitor : public rclcpp::Node
{
public:
  explicit BoardMonitor(const rclcpp::NodeOptions & options)
  : rclcpp::Node("board_monitor", options)
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/overhead_camera/image_raw");
    const auto service_name = declare_parameter<std::string>(
      "service_name", "/perception/get_board_state");

    // Frames arrive on their own callback group, so a query is answered while
    // frames keep arriving. The service stays in the default group: on Lyrical
    // with rmw_zenoh, a service placed in a separately created callback group is
    // never executed (subscriptions in such groups are).
    auto image_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = image_group;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(msg);
      }, sub_opts);

    service_ = create_service<GetBoardState>(
      service_name,
      [this](const GetBoardState::Request::SharedPtr req, GetBoardState::Response::SharedPtr res) {
        handle(*req, *res);
      },
      rclcpp::ServicesQoS());

    RCLCPP_INFO(get_logger(), "Serving %s from %s", service_name.c_str(), image_topic.c_str());
  }

private:
  void handle(const GetBoardState::Request & req, GetBoardState::Response & res)
  {
    RCLCPP_DEBUG(get_logger(), "GetBoardState request (max_frame_age_s=%.2f)", req.max_frame_age_s);
    sensor_msgs::msg::Image::ConstSharedPtr frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame = latest_;
    }

    res.state.squares.fill(BoardState::UNKNOWN);
    res.state.confidence.fill(0.0f);
    res.state.board_detected = false;

    if (!frame) {
      res.result = GetBoardState::Response::RESULT_NO_IMAGE;
      return;
    }
    const double age = (now() - rclcpp::Time(frame->header.stamp)).seconds();
    if (req.max_frame_age_s > 0.0f && age > req.max_frame_age_s) {
      res.result = GetBoardState::Response::RESULT_NO_IMAGE;
      return;
    }

    // The answer carries the frame's stamp and frame, so consumers can line the
    // facts up with the pixels they came from.
    res.state.header = frame->header;

    const auto detection = detect_board(*frame);
    if (!detection.board_found) {
      res.result = GetBoardState::Response::RESULT_NO_BOARD;
      return;
    }
    res.state.board_detected = true;
    res.state.squares = detection.squares;
    res.state.confidence = detection.confidence;
    res.result = GetBoardState::Response::RESULT_OK;
  }

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<GetBoardState>::SharedPtr service_;
};

}  // namespace chessbot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(chessbot_perception::BoardMonitor)
