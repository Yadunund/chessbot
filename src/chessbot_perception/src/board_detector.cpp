// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include "chessbot_perception/board_detector.hpp"

namespace chessbot_perception
{

BoardDetection detect_board(const sensor_msgs::msg::Image & /*frame*/)
{
  // TODO(chessbot): board localisation and per-square classification.
  return BoardDetection{};
}

}  // namespace chessbot_perception
