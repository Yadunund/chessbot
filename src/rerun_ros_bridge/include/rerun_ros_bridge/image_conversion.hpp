// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#ifndef RERUN_ROS_BRIDGE__IMAGE_CONVERSION_HPP_
#define RERUN_ROS_BRIDGE__IMAGE_CONVERSION_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rerun_ros_bridge
{

/// How a ROS image encoding maps onto a Rerun image.
enum class PixelLayout
{
  kColor,   // Image with a colour model
  kDepth,   // DepthImage
};

struct EncodingInfo
{
  PixelLayout layout;
  uint8_t channels;
  uint8_t bytes_per_channel;
  // Rerun colour model as its numeric value (rerun::ColorModel), for colour layouts.
  uint8_t color_model;
  // Rerun channel datatype as its numeric value (rerun::ChannelDatatype).
  uint8_t channel_datatype;
  // Units per metre, for depth layouts (1000 for millimetres, 1 for metres).
  float depth_meter;
};

/// Look up how to interpret a sensor_msgs/Image encoding.
/// Returns nullopt for encodings the bridge does not handle (e.g. Bayer).
std::optional<EncodingInfo> lookup_encoding(const std::string & encoding);

/// A packed image buffer, ready to hand to the Rerun SDK.
struct PackedImage
{
  std::vector<uint8_t> bytes;
  uint32_t width;
  uint32_t height;
};

/// Copy ROS image rows into a tightly packed, little-endian buffer.
///
/// Handles three details of sensor_msgs/Image that a naive copy gets wrong:
/// - `step` may exceed width * pixel size (row padding), so rows are copied
///   individually;
/// - multi-byte data may be big-endian, in which case it is byte-swapped;
/// - large frames can be downscaled by an integer stride so the live stream
///   stays small (nearest neighbour; `max_width` of 0 disables downscaling).
PackedImage pack_image(
  const uint8_t * data, size_t data_size, uint32_t width, uint32_t height, uint32_t step,
  const EncodingInfo & info, bool is_bigendian, uint32_t max_width);

}  // namespace rerun_ros_bridge

#endif  // RERUN_ROS_BRIDGE__IMAGE_CONVERSION_HPP_
