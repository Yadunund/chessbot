// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "chessbot_perception/square_classifier.hpp"

using chessbot_perception::BoardPose;
using chessbot_perception::Camera;
using chessbot_perception::ImageView;
using chessbot_perception::labels_from_fen;
using chessbot_perception::Occupancy;
using chessbot_perception::SquareClassifier;

namespace
{

// A camera 0.5 m above the board centre looking straight down, and a synthetic image
// painted from occupancy: squares in two colours, pieces as white or black discs.
struct Scene
{
  BoardPose board{{0.0, 0.0, 0.0}, 0.0, 0.02625};
  Camera camera;
  int width{640}, height{640};
  std::vector<uint8_t> pixels;

  Scene()
  {
    camera.fx = camera.fy = 1200.0;
    camera.cx = width / 2.0;
    camera.cy = height / 2.0;
    // Base -> camera: looking down (camera z = -base z), camera x = base x, camera y = -base y.
    camera.rotation = {1, 0, 0, 0, -1, 0, 0, 0, -1};
    const double c = 4 * board.square;
    camera.translation = {-c, c, 0.5};
  }

  ImageView paint(const std::array<Occupancy, 64> & occupancy)
  {
    pixels.assign(width * height * 3, 0);
    // Invert the projection for each pixel onto the board plane.
    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        const double x = (u - camera.cx) / camera.fx * 0.5 + 4 * board.square;
        const double y = -(v - camera.cy) / camera.fy * 0.5 + 4 * board.square;
        uint8_t r = 180, g = 150, b = 120;  // table
        const int file = static_cast<int>(std::floor(x / board.square));
        const int rank = static_cast<int>(std::floor(y / board.square));
        if (file >= 0 && file < 8 && rank >= 0 && rank < 8) {
          const bool dark = (file + rank) % 2 == 0;
          r = dark ? 117 : 217; g = dark ? 92 : 199; b = dark ? 66 : 163;
          const double dx = x - (file + 0.5) * board.square, dy = y - (rank + 0.5) * board.square;
          const auto piece = occupancy[file + 8 * rank];
          if (piece != Occupancy::kEmpty && dx * dx + dy * dy < 0.0075 * 0.0075) {
            const uint8_t shade = piece == Occupancy::kWhite ? 230 : 30;
            r = g = b = shade;
          }
        }
        uint8_t * p = &pixels[(v * width + u) * 3];
        p[0] = r; p[1] = g; p[2] = b;
      }
    }
    return ImageView{pixels.data(), width, height, width * 3, false};
  }
};

}  // namespace

TEST(SquareClassifier, ParsesFenPlacement)
{
  const auto labels = labels_from_fen("rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1");
  ASSERT_TRUE(labels);
  EXPECT_EQ((*labels)[0], Occupancy::kWhite);   // a1
  EXPECT_EQ((*labels)[12], Occupancy::kEmpty);  // e2
  EXPECT_EQ((*labels)[28], Occupancy::kWhite);  // e4
  EXPECT_EQ((*labels)[63], Occupancy::kBlack);  // h8
  EXPECT_FALSE(labels_from_fen("8/8/8"));
}

TEST(SquareClassifier, LearnsFromStartAndReadsAMove)
{
  Scene scene;
  SquareClassifier classifier;
  const auto start = *labels_from_fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_FALSE(classifier.has_references());
  classifier.learn(classifier.measure(scene.paint(start), scene.camera, scene.board), start);
  EXPECT_TRUE(classifier.has_references());

  const auto after = *labels_from_fen("rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2");
  std::array<Occupancy, 64> seen;
  std::array<float, 64> confidence;
  classifier.classify(classifier.measure(scene.paint(after), scene.camera, scene.board), seen, confidence);
  for (int i = 0; i < 64; ++i) {
    EXPECT_EQ(seen[i], after[i]) << "square " << i;
    EXPECT_GT(confidence[i], 0.2f) << "square " << i;
  }
}
