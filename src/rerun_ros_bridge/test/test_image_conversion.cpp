// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <vector>

#include "rerun_ros_bridge/image_conversion.hpp"

using rerun_ros_bridge::lookup_encoding;
using rerun_ros_bridge::pack_image;

TEST(ImageConversion, UnknownEncodingIsRejected)
{
  EXPECT_FALSE(lookup_encoding("bayer_rggb8").has_value());
  EXPECT_TRUE(lookup_encoding("rgb8").has_value());
}

TEST(ImageConversion, RowPaddingIsStripped)
{
  // 2x2 rgb8 with one byte of padding per row (step = 7).
  std::vector<uint8_t> data = {1, 2, 3, 4, 5, 6, 0, 7, 8, 9, 10, 11, 12, 0};
  auto info = *lookup_encoding("rgb8");
  auto out = pack_image(data.data(), data.size(), 2, 2, 7, info, false, 0);
  ASSERT_EQ(out.width, 2u);
  ASSERT_EQ(out.height, 2u);
  EXPECT_EQ(out.bytes, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));
}

TEST(ImageConversion, BigEndianIsSwapped)
{
  std::vector<uint8_t> data = {0x01, 0x02};
  auto info = *lookup_encoding("16UC1");
  auto out = pack_image(data.data(), data.size(), 1, 1, 2, info, true, 0);
  EXPECT_EQ(out.bytes, (std::vector<uint8_t>{0x02, 0x01}));
}

TEST(ImageConversion, DownscaleByStride)
{
  // 4x2 mono8 downscaled to max width 2 -> stride 2 -> 2x1.
  std::vector<uint8_t> data = {1, 2, 3, 4, 5, 6, 7, 8};
  auto info = *lookup_encoding("mono8");
  auto out = pack_image(data.data(), data.size(), 4, 2, 4, info, false, 2);
  ASSERT_EQ(out.width, 2u);
  ASSERT_EQ(out.height, 1u);
  EXPECT_EQ(out.bytes, (std::vector<uint8_t>{1, 3}));
}

TEST(ImageConversion, MalformedStepIsRejected)
{
  std::vector<uint8_t> data(6, 0);
  auto info = *lookup_encoding("rgb8");
  auto out = pack_image(data.data(), data.size(), 2, 2, 3, info, false, 0);
  EXPECT_EQ(out.width, 0u);
}
