// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#ifndef CHESSBOT_PERCEPTION__SQUARE_CLASSIFIER_HPP_
#define CHESSBOT_PERCEPTION__SQUARE_CLASSIFIER_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace chessbot_perception
{

/// What occupies a square, as in chessbot_interfaces/BoardState.
enum class Occupancy : uint8_t { kEmpty = 0, kWhite = 1, kBlack = 2, kUnknown = 3 };

/// Pinhole camera and its pose: projects base-frame points to pixels.
struct Camera
{
  double fx{0}, fy{0}, cx{0}, cy{0};
  // Rotation (row-major) and translation taking base-frame points into the camera optical frame.
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::array<double, 3> translation{0, 0, 0};

  /// Pixel coordinates, or nothing when the point is behind the camera.
  std::optional<std::array<double, 2>> project(double x, double y, double z) const;
};

/// Where the board is, in the robot base frame (the calibration profile's board section).
struct BoardPose
{
  std::array<double, 3> origin{0, 0, 0};  // outer corner of a1
  double yaw{0};
  double square{0.02625};

  /// Base-frame centre of a square, index file + 8 * rank.
  std::array<double, 3> square_centre(int index) const;
};

/// A packed RGB image view (row padding allowed).
struct ImageView
{
  const uint8_t * data{nullptr};
  int width{0};
  int height{0};
  int step{0};  // bytes per row
  bool bgr{false};
};

/// Mean colour of a patch, in CIE L*a*b*.
using Colour = std::array<double, 3>;

/// Classifies squares as empty / white piece / black piece by comparing each square's
/// colour with references learned from positions whose contents are known.
///
/// References are kept per square and per class, and also pooled over light and dark
/// squares, so a square that has never held, say, a black piece still has a reference.
class SquareClassifier
{
public:
  struct Options
  {
    double sample_height{0.003};  // m above the board: low, so leaning pieces nearby do not cover it
    double sample_radius{0.004};  // m
    int samples_per_axis{7};
    double learning_rate{0.5};    // weight of a new observation in a reference
  };

  SquareClassifier();
  explicit SquareClassifier(Options options);

  /// Mean colour around each square centre; nullopt where the patch is outside the image.
  std::array<std::optional<Colour>, 64> measure(const ImageView & image, const Camera & camera, const BoardPose & board) const;

  /// Update references from colours whose occupancy is known.
  void learn(const std::array<std::optional<Colour>, 64> & colours, const std::array<Occupancy, 64> & labels);

  /// Classify colours. Confidence is 0 (a tie) to 1 (clearly one class).
  void classify(
    const std::array<std::optional<Colour>, 64> & colours,
    std::array<Occupancy, 64> & occupancy, std::array<float, 64> & confidence) const;

  bool has_references() const {return learned_;}

private:
  struct Reference
  {
    Colour colour{0, 0, 0};
    bool valid{false};
  };
  static int parity(int index) {return ((index % 8) + (index / 8)) % 2;}  // 0: dark (a1), 1: light
  void blend(Reference & ref, const Colour & colour) const;

  Options options_;
  std::array<std::array<Reference, 3>, 64> per_square_{};
  std::array<std::array<Reference, 3>, 2> per_parity_{};
  bool learned_{false};
};

/// Occupancy labels from the piece placement field of a FEN.
std::optional<std::array<Occupancy, 64>> labels_from_fen(const std::string & fen);

}  // namespace chessbot_perception

#endif  // CHESSBOT_PERCEPTION__SQUARE_CLASSIFIER_HPP_
