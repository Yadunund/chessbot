// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#ifndef CHESSBOT_PERCEPTION__BOARD_DETECTOR_HPP_
#define CHESSBOT_PERCEPTION__BOARD_DETECTOR_HPP_

#include <array>
#include <cstdint>

#include <sensor_msgs/msg/image.hpp>

namespace chessbot_perception
{

struct BoardDetection
{
  bool board_found{false};
  // Same encoding and indexing as chessbot_interfaces/BoardState.
  std::array<uint8_t, 64> squares{};
  std::array<float, 64> confidence{};
};

/// Locate the board in a frame and classify each square.
///
/// Not implemented yet: returns "no board" until the board model and the
/// detector exist. Kept behind this function so the node, its contract and its
/// tests do not change when the real detector lands.
BoardDetection detect_board(const sensor_msgs::msg::Image & frame);

}  // namespace chessbot_perception

#endif  // CHESSBOT_PERCEPTION__BOARD_DETECTOR_HPP_
