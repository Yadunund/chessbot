// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

// Perception: camera frames in, board-level facts out, on request.
//
// Answers chessbot_interfaces/GetBoardState with the occupancy of each square
// (empty, white piece, black piece) and a confidence.
//
// Where the squares are in the image comes from what is already known: the board
// pose from the calibration profile in the key-value store, the camera model from
// camera_info, and the camera pose from TF. Nothing is detected in the image.
//
// How a square looks when empty or occupied is learned whenever the believed
// position can be trusted: each time the game state enters the human's turn (the
// robot has finished and parked, or a new game started), the next frame is
// labelled with that position.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <chessbot_interfaces/msg/board_state.hpp>
#include <chessbot_interfaces/msg/game_state.hpp>
#include <chessbot_interfaces/srv/get_board_state.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "chessbot_perception/kv_client.hpp"
#include "chessbot_perception/square_classifier.hpp"

namespace chessbot_perception
{

using chessbot_interfaces::msg::BoardState;
using chessbot_interfaces::msg::GameState;
using chessbot_interfaces::srv::GetBoardState;

class BoardMonitor : public rclcpp::Node
{
public:
  explicit BoardMonitor(const rclcpp::NodeOptions & options)
  : rclcpp::Node("board_monitor", options)
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/overhead_camera/image_raw");
    const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/overhead_camera/camera_info");
    const auto service_name = declare_parameter<std::string>("service_name", "/perception/get_board_state");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    calibration_key_ = declare_parameter<std::string>("calibration_key", "chessbot/kv/calibration");
    kv_ = std::make_unique<KvClient>(declare_parameter<std::string>("zenoh_endpoint", "tcp/127.0.0.1:7447"));
    // Frames right after the phase change may still show the arm settling.
    reference_delay_ = rclcpp::Duration::from_seconds(declare_parameter<double>("reference_delay_s", 1.0));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // Frames arrive on their own callback group, so a query is answered while
    // frames keep arriving. The service stays in the default group: on Lyrical
    // with rmw_zenoh, a service placed in a separately created callback group is
    // never executed (subscriptions in such groups are).
    auto sensor_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sensor_opts;
    sensor_opts.callback_group = sensor_group;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(std::move(msg));}, sensor_opts);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        info_ = std::move(msg);
      }, sensor_opts);
    game_sub_ = create_subscription<GameState>(
      "/chessbot/game_state", rclcpp::QoS(1).reliable().transient_local(),
      [this](GameState::ConstSharedPtr msg) {on_game_state(*msg);}, sensor_opts);

    service_ = create_service<GetBoardState>(
      service_name,
      [this](const GetBoardState::Request::SharedPtr req, GetBoardState::Response::SharedPtr res) {
        handle(*req, *res);
      },
      rclcpp::ServicesQoS());

    RCLCPP_INFO(get_logger(), "Serving %s from %s", service_name.c_str(), image_topic.c_str());
  }

private:
  // --- inputs -----------------------------------------------------------------------

  void on_game_state(const GameState & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool human_turn = msg.phase == GameState::PHASE_HUMAN_TURN;
    if (human_turn && (last_phase_ != GameState::PHASE_HUMAN_TURN || msg.fen != last_fen_)) {
      pending_labels_ = labels_from_fen(msg.fen);
      pending_after_ = now() + reference_delay_;
    }
    last_phase_ = msg.phase;
    last_fen_ = msg.fen;
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::optional<std::array<Occupancy, 64>> labels;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = msg;
      if (pending_labels_ && rclcpp::Time(msg->header.stamp) >= pending_after_) {
        labels = pending_labels_;
        pending_labels_.reset();
      }
    }
    if (!labels) {
      return;
    }
    const auto colours = measure(*msg);
    if (!colours) {
      // Not ready (no calibration, camera model or transform yet): try the next frame.
      std::lock_guard<std::mutex> lock(mutex_);
      pending_labels_ = labels;
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    classifier_.learn(*colours, *labels);
    RCLCPP_INFO(get_logger(), "Learned square appearance from the believed position");
  }

  // --- geometry ---------------------------------------------------------------------

  std::optional<BoardPose> board_pose()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (board_) {
        return board_;
      }
    }
    const auto text = kv_->get(calibration_key_);
    if (!text) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No calibration at %s", calibration_key_.c_str());
      return std::nullopt;
    }
    try {
      const auto board = nlohmann::json::parse(*text).at("board");
      BoardPose pose;
      for (int i = 0; i < 3; ++i) {
        pose.origin[i] = board.at("origin_xyz").at(i).get<double>();
      }
      pose.yaw = board.at("yaw_rad").get<double>();
      pose.square = board.at("square_size_m").get<double>();
      std::lock_guard<std::mutex> lock(mutex_);
      board_ = pose;
      return pose;
    } catch (const nlohmann::json::exception & e) {
      RCLCPP_ERROR(get_logger(), "Unreadable calibration: %s", e.what());
      return std::nullopt;
    }
  }

  std::optional<Camera> camera(const sensor_msgs::msg::Image & frame)
  {
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      info = info_;
    }
    if (!info || info->k[0] <= 0.0) {
      return std::nullopt;
    }
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(frame.header.frame_id, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No camera pose: %s", e.what());
      return std::nullopt;
    }
    Camera cam;
    cam.fx = info->k[0];
    cam.fy = info->k[4];
    cam.cx = info->k[2];
    cam.cy = info->k[5];
    const auto & q = t.transform.rotation;
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    cam.rotation = {
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
    cam.translation = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};
    return cam;
  }

  std::optional<std::array<std::optional<Colour>, 64>> measure(const sensor_msgs::msg::Image & frame)
  {
    if (frame.encoding != "rgb8" && frame.encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported encoding %s", frame.encoding.c_str());
      return std::nullopt;
    }
    const auto board = board_pose();
    const auto cam = camera(frame);
    if (!board || !cam) {
      return std::nullopt;
    }
    ImageView view{frame.data.data(), static_cast<int>(frame.width), static_cast<int>(frame.height),
      static_cast<int>(frame.step), frame.encoding == "bgr8"};
    return classifier_.measure(view, *cam, *board);
  }

  // --- service ----------------------------------------------------------------------

  void handle(const GetBoardState::Request & req, GetBoardState::Response & res)
  {
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

    const auto colours = measure(*frame);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!colours || !classifier_.has_references()) {
      // The board cannot be located (no calibration or camera model) or has never been seen in a known position.
      res.result = GetBoardState::Response::RESULT_NO_BOARD;
      return;
    }
    std::array<Occupancy, 64> occupancy;
    std::array<float, 64> confidence;
    classifier_.classify(*colours, occupancy, confidence);
    for (int i = 0; i < 64; ++i) {
      res.state.squares[i] = static_cast<uint8_t>(occupancy[i]);
      res.state.confidence[i] = confidence[i];
    }
    res.state.board_detected = true;
    res.result = GetBoardState::Response::RESULT_OK;
  }

  std::string base_frame_;
  std::string calibration_key_;
  rclcpp::Duration reference_delay_{0, 0};
  std::unique_ptr<KvClient> kv_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::optional<BoardPose> board_;
  SquareClassifier classifier_;
  uint8_t last_phase_{255};
  std::string last_fen_;
  std::optional<std::array<Occupancy, 64>> pending_labels_;
  rclcpp::Time pending_after_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<GameState>::SharedPtr game_sub_;
  rclcpp::Service<GetBoardState>::SharedPtr service_;
};

}  // namespace chessbot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(chessbot_perception::BoardMonitor)
