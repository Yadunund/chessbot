// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

// A generic ROS 2 -> Rerun bridge, as a composable node.
//
// Load it into the same container as the image publishers with
// use_intra_process_comms so frames arrive by pointer. It serves a Rerun gRPC
// stream that the stock web or native viewer connects to, and can archive the
// same data to an .rrd file.
//
// Nothing here is specific to any application: what to log is configured by
// parameters (topic -> entity path). Topics of any type can be logged as text:
// their type is discovered at runtime and rendered through introspection.

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rerun.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rerun_ros_bridge/image_conversion.hpp"
#include "rerun_ros_bridge/message_to_yaml.hpp"

namespace rerun_ros_bridge
{

class RerunBridge : public rclcpp::Node
{
public:
  explicit RerunBridge(const rclcpp::NodeOptions & options)
  : rclcpp::Node("rerun_bridge", options),
    rec_(declare_parameter<std::string>("application_id", "ros"))
  {
    const bool serve = declare_parameter<bool>("serve_grpc", true);
    const auto bind_ip = declare_parameter<std::string>("grpc_bind", "0.0.0.0");
    const auto port = static_cast<uint16_t>(declare_parameter<int>("grpc_port", 9876));
    const auto memory_limit = declare_parameter<std::string>("server_memory_limit", "512MiB");
    const bool save = declare_parameter<bool>("save_rrd", false);
    const auto recording_dir = declare_parameter<std::string>("recording_dir", "");

    image_period_ = std::chrono::duration<double>(
      1.0 / std::max(0.01, declare_parameter<double>("image_rate_hz", 5.0)));
    image_max_width_ = static_cast<uint32_t>(declare_parameter<int>("image_max_width", 640));

    setup_sinks(serve, bind_ip, port, memory_limit, save, recording_dir);

    // Each subscription gets its own callback group so a slow image conversion
    // never delays joint states when the container runs a multi-threaded
    // executor.
    auto image_topics = declare_parameter<std::vector<std::string>>(
      "image_topics", std::vector<std::string>{});
    auto image_entities = declare_parameter<std::vector<std::string>>(
      "image_entities", std::vector<std::string>{});
    subscribe_images(image_topics, image_entities);

    auto compressed_topics = declare_parameter<std::vector<std::string>>(
      "compressed_image_topics", std::vector<std::string>{});
    auto compressed_entities = declare_parameter<std::vector<std::string>>(
      "compressed_image_entities", std::vector<std::string>{});
    subscribe_compressed_images(compressed_topics, compressed_entities);

    text_topics_ = declare_parameter<std::vector<std::string>>("text_topics", std::vector<std::string>{});
    text_entities_ = declare_parameter<std::vector<std::string>>("text_entities", std::vector<std::string>{});
    // "log" appends each message to a TextLog; "document" shows the latest as a TextDocument.
    text_modes_ = declare_parameter<std::vector<std::string>>("text_modes", std::vector<std::string>{});
    if (!text_topics_.empty()) {
      // Types are only known once someone publishes, so keep looking until every topic is found.
      discover_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {discover_text_topics();});
    }

    const auto joint_topic = declare_parameter<std::string>("joint_states_topic", "");
    joint_entity_ = declare_parameter<std::string>("joint_states_entity", "joints");
    if (!joint_topic.empty()) {
      auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions opts;
      opts.callback_group = group;
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_state(*msg);}, opts);
    }
  }

private:
  void setup_sinks(
    bool serve, const std::string & bind_ip, uint16_t port, const std::string & memory_limit,
    bool save, const std::string & recording_dir)
  {
    // Sinks hold string_views into their own members (and into ours), so they
    // are kept as named locals and passed to set_sinks directly rather than
    // converted to LogSink and stored.
    if (save && !recording_dir.empty()) {
      std::filesystem::create_directories(recording_dir);
      const auto stamp = std::chrono::system_clock::now().time_since_epoch();
      rrd_path_ = (std::filesystem::path(recording_dir) /
        ("recording_" + std::to_string(
          std::chrono::duration_cast<std::chrono::seconds>(stamp).count()) + ".rrd")).string();
    }
    // Any origin may connect: the viewer tab is served from a different port.
    const rerun::GrpcServerSink grpc(bind_ip, port, memory_limit,
      rerun::PlaybackBehavior::OldestFirst, {"*"});
    const rerun::FileSink file{rrd_path_};

    rerun::Error err;
    if (serve && !rrd_path_.empty()) {
      err = rec_.set_sinks(grpc, file);
    } else if (serve) {
      err = rec_.set_sinks(grpc);
    } else if (!rrd_path_.empty()) {
      err = rec_.set_sinks(file);
    }
    if (err.is_err()) {
      RCLCPP_ERROR(get_logger(), "Failed to set up Rerun sinks: %s", err.description.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Rerun: grpc=%s port=%u rrd=%s", serve ? "on" : "off", port,
      rrd_path_.empty() ? "off" : rrd_path_.c_str());
  }

  void subscribe_images(
    const std::vector<std::string> & topics, const std::vector<std::string> & entities)
  {
    for (size_t i = 0; i < topics.size(); ++i) {
      const std::string entity = i < entities.size() ? entities[i] : topics[i];
      auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions opts;
      opts.callback_group = group;
      image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, entity](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(entity, *msg);},
        opts));
    }
  }

  void subscribe_compressed_images(
    const std::vector<std::string> & topics, const std::vector<std::string> & entities)
  {
    for (size_t i = 0; i < topics.size(); ++i) {
      const std::string entity = i < entities.size() ? entities[i] : topics[i];
      auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions opts;
      opts.callback_group = group;
      compressed_subs_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, entity](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
          on_compressed_image(entity, *msg);
        }, opts));
    }
  }

  // Rate-limit per entity so a 30 fps camera does not flood the viewer.
  bool due(const std::string & entity)
  {
    std::lock_guard<std::mutex> lock(rate_mutex_);
    const auto now = std::chrono::steady_clock::now();
    auto it = last_logged_.find(entity);
    if (it != last_logged_.end() && now - it->second < image_period_) {
      return false;
    }
    last_logged_[entity] = now;
    return true;
  }

  // The timeline follows this node's clock: simulation time from /clock when use_sim_time
  // is set, shown as elapsed time, otherwise wall-clock time, shown as a date.
  void set_time()
  {
    const auto clock = get_clock();
    const int64_t nanos = clock->now().nanoseconds();
    if (clock->ros_time_is_active()) {
      rec_.set_time_duration_nanos("ros_time", nanos);
    } else {
      rec_.set_time_timestamp_nanos_since_epoch("ros_time", nanos);
    }
  }

  void on_image(const std::string & entity, const sensor_msgs::msg::Image & msg)
  {
    if (!due(entity)) {
      return;
    }
    const auto info = lookup_encoding(msg.encoding);
    if (!info) {
      RCLCPP_WARN_ONCE(get_logger(), "Unsupported image encoding '%s' on %s", msg.encoding.c_str(),
        entity.c_str());
      return;
    }
    auto packed = pack_image(msg.data.data(), msg.data.size(), msg.width, msg.height, msg.step,
      *info, msg.is_bigendian, image_max_width_);
    if (packed.width == 0) {
      RCLCPP_WARN_ONCE(get_logger(), "Malformed image on %s", entity.c_str());
      return;
    }

    set_time();
    const rerun::WidthHeight resolution{packed.width, packed.height};
    const auto datatype = static_cast<rerun::encodings::ChannelDatatype>(info->channel_datatype);
    auto bytes = rerun::Collection<uint8_t>::take_ownership(std::move(packed.bytes));
    if (info->layout == PixelLayout::kDepth) {
      rec_.log(entity, rerun::DepthImage(bytes.data(), resolution, datatype)
        .with_meter(info->depth_meter));
    } else {
      rec_.log(entity, rerun::Image(std::move(bytes), resolution,
        static_cast<rerun::encodings::ColorModel>(info->color_model), datatype));
    }
  }

  void on_compressed_image(const std::string & entity, const sensor_msgs::msg::CompressedImage & msg)
  {
    if (!due(entity)) {
      return;
    }
    set_time();
    // ROS formats look like "jpeg" or "rgb8; jpeg compressed bgr8".
    const bool png = msg.format.find("png") != std::string::npos;
    // Borrowed, not copied: log() serialises synchronously before returning.
    rec_.log(entity, rerun::EncodedImage::from_bytes(
      rerun::Collection<uint8_t>::borrow(msg.data.data(), msg.data.size()),
      png ? rerun::MediaType::png() : rerun::MediaType::jpeg()));
  }

  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    set_time();
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
      rec_.log(joint_entity_ + "/" + msg.name[i], rerun::Scalars(msg.position[i]));
    }
  }

  void discover_text_topics()
  {
    const auto graph = get_topic_names_and_types();
    bool all_found = true;
    for (size_t i = 0; i < text_topics_.size(); ++i) {
      const auto & topic = text_topics_[i];
      if (text_subs_.count(topic)) {
        continue;
      }
      auto it = graph.find(topic);
      if (it == graph.end() || it->second.empty()) {
        all_found = false;
        continue;
      }
      const std::string type = it->second.front();
      const std::string entity = i < text_entities_.size() ? text_entities_[i] : topic;
      const bool as_log = !(i < text_modes_.size() && text_modes_[i] == "document");
      std::shared_ptr<MessageToYaml> renderer;
      try {
        renderer = std::make_shared<MessageToYaml>(type);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Cannot log %s (%s): %s", topic.c_str(), type.c_str(), e.what());
        text_subs_[topic] = nullptr;  // do not retry a type we cannot load
        continue;
      }
      // Reliable and transient-local, so a latched topic's current value arrives on subscribe.
      const auto qos = rclcpp::QoS(10).reliable().transient_local();
      text_subs_[topic] = create_generic_subscription(
        topic, type, qos,
        [this, entity, as_log, renderer](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
          const std::string text = renderer->render(*msg);
          set_time();
          if (as_log) {
            rec_.log(entity, rerun::TextLog(text));
          } else {
            // A fenced block keeps the YAML's line breaks and indentation.
            rec_.log(entity, rerun::TextDocument("```yaml\n" + text + "\n```")
              .with_media_type(rerun::MediaType::markdown()));
          }
        });
      RCLCPP_INFO(get_logger(), "Logging %s (%s) as text at %s", topic.c_str(), type.c_str(), entity.c_str());
    }
    if (all_found) {
      discover_timer_->cancel();
    }
  }

  rerun::RecordingStream rec_;
  std::vector<std::string> text_topics_;
  std::vector<std::string> text_entities_;
  std::vector<std::string> text_modes_;
  rclcpp::TimerBase::SharedPtr discover_timer_;
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> text_subs_;
  std::string rrd_path_;
  std::chrono::duration<double> image_period_{0.2};
  uint32_t image_max_width_{640};
  std::string joint_entity_;

  std::mutex rate_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_logged_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_subs_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

}  // namespace rerun_ros_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(rerun_ros_bridge::RerunBridge)
