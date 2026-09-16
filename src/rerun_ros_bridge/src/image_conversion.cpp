// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include "rerun_ros_bridge/image_conversion.hpp"

#include <algorithm>
#include <unordered_map>

#include <rerun.hpp>

namespace rerun_ros_bridge
{

namespace
{
uint8_t cm(rerun::encodings::ColorModel model) {return static_cast<uint8_t>(model);}
uint8_t dt(rerun::encodings::ChannelDatatype type) {return static_cast<uint8_t>(type);}
}  // namespace

std::optional<EncodingInfo> lookup_encoding(const std::string & encoding)
{
  using rerun::encodings::ChannelDatatype;
  using rerun::encodings::ColorModel;
  static const std::unordered_map<std::string, EncodingInfo> table = {
    {"rgb8", {PixelLayout::kColor, 3, 1, cm(ColorModel::RGB), dt(ChannelDatatype::U8), 0.0f}},
    {"bgr8", {PixelLayout::kColor, 3, 1, cm(ColorModel::BGR), dt(ChannelDatatype::U8), 0.0f}},
    {"rgba8", {PixelLayout::kColor, 4, 1, cm(ColorModel::RGBA), dt(ChannelDatatype::U8), 0.0f}},
    {"bgra8", {PixelLayout::kColor, 4, 1, cm(ColorModel::BGRA), dt(ChannelDatatype::U8), 0.0f}},
    {"mono8", {PixelLayout::kColor, 1, 1, cm(ColorModel::L), dt(ChannelDatatype::U8), 0.0f}},
    {"8UC1", {PixelLayout::kColor, 1, 1, cm(ColorModel::L), dt(ChannelDatatype::U8), 0.0f}},
    {"mono16", {PixelLayout::kColor, 1, 2, cm(ColorModel::L), dt(ChannelDatatype::U16), 0.0f}},
    {"16UC1", {PixelLayout::kDepth, 1, 2, 0, dt(ChannelDatatype::U16), 1000.0f}},
    {"32FC1", {PixelLayout::kDepth, 1, 4, 0, dt(ChannelDatatype::F32), 1.0f}},
  };
  auto it = table.find(encoding);
  if (it == table.end()) {
    return std::nullopt;
  }
  return it->second;
}

PackedImage pack_image(
  const uint8_t * data, size_t data_size, uint32_t width, uint32_t height, uint32_t step,
  const EncodingInfo & info, bool is_bigendian, uint32_t max_width)
{
  const size_t pixel_size = static_cast<size_t>(info.channels) * info.bytes_per_channel;
  uint32_t stride = 1;
  if (max_width > 0 && width > max_width) {
    stride = (width + max_width - 1) / max_width;
  }

  PackedImage out;
  out.width = width / stride;
  out.height = height / stride;
  out.bytes.resize(static_cast<size_t>(out.width) * out.height * pixel_size);

  // A malformed message (step or size too small) yields an empty image rather
  // than reading past the buffer.
  if (step < width * pixel_size || data_size < static_cast<size_t>(step) * height) {
    out.width = 0;
    out.height = 0;
    out.bytes.clear();
    return out;
  }

  uint8_t * dst = out.bytes.data();
  for (uint32_t y = 0; y < out.height; ++y) {
    const uint8_t * row = data + static_cast<size_t>(y) * stride * step;
    if (stride == 1) {
      std::copy_n(row, out.width * pixel_size, dst);
      dst += out.width * pixel_size;
    } else {
      for (uint32_t x = 0; x < out.width; ++x) {
        std::copy_n(row + static_cast<size_t>(x) * stride * pixel_size, pixel_size, dst);
        dst += pixel_size;
      }
    }
  }

  if (is_bigendian && info.bytes_per_channel > 1) {
    for (size_t i = 0; i + info.bytes_per_channel <= out.bytes.size(); i += info.bytes_per_channel) {
      std::reverse(out.bytes.begin() + i, out.bytes.begin() + i + info.bytes_per_channel);
    }
  }
  return out;
}

}  // namespace rerun_ros_bridge
