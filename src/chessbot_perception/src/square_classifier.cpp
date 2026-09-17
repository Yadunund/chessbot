// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include "chessbot_perception/square_classifier.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

namespace chessbot_perception
{

std::optional<std::array<double, 2>> Camera::project(double x, double y, double z) const
{
  const auto & r = rotation;
  const double cx_ = r[0] * x + r[1] * y + r[2] * z + translation[0];
  const double cy_ = r[3] * x + r[4] * y + r[5] * z + translation[1];
  const double cz_ = r[6] * x + r[7] * y + r[8] * z + translation[2];
  if (cz_ <= 1e-6) {
    return std::nullopt;
  }
  return std::array<double, 2>{fx * cx_ / cz_ + cx, fy * cy_ / cz_ + cy};
}

std::array<double, 3> BoardPose::square_centre(int index) const
{
  const double bx = (index % 8 + 0.5) * square;
  const double by = (index / 8 + 0.5) * square;
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {origin[0] + c * bx - s * by, origin[1] + s * bx + c * by, origin[2]};
}

namespace
{

double srgb_to_linear(double v)
{
  v /= 255.0;
  return v <= 0.04045 ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4);
}

Colour rgb_to_lab(double r, double g, double b)
{
  r = srgb_to_linear(r);
  g = srgb_to_linear(g);
  b = srgb_to_linear(b);
  // sRGB D65 -> XYZ, normalised by the white point.
  double x = (0.4124 * r + 0.3576 * g + 0.1805 * b) / 0.95047;
  double y = (0.2126 * r + 0.7152 * g + 0.0722 * b);
  double z = (0.0193 * r + 0.1192 * g + 0.9505 * b) / 1.08883;
  auto f = [](double t) {return t > 0.008856 ? std::cbrt(t) : 7.787 * t + 16.0 / 116.0;};
  const double fx = f(x), fy = f(y), fz = f(z);
  return {116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz)};
}

double distance(const Colour & a, const Colour & b)
{
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

}  // namespace

SquareClassifier::SquareClassifier()
: SquareClassifier(Options{}) {}

SquareClassifier::SquareClassifier(Options options)
: options_(options) {}

std::array<std::optional<Colour>, 64> SquareClassifier::measure(
  const ImageView & image, const Camera & camera, const BoardPose & board) const
{
  std::array<std::optional<Colour>, 64> out;
  const int n = std::max(options_.samples_per_axis, 1);
  for (int index = 0; index < 64; ++index) {
    const auto centre = board.square_centre(index);
    double sr = 0, sg = 0, sb = 0;
    int count = 0;
    bool outside = false;
    for (int i = 0; i < n && !outside; ++i) {
      for (int j = 0; j < n; ++j) {
        const double u = n == 1 ? 0.0 : (2.0 * i / (n - 1) - 1.0);
        const double v = n == 1 ? 0.0 : (2.0 * j / (n - 1) - 1.0);
        if (u * u + v * v > 1.0) {
          continue;
        }
        const auto pixel = camera.project(
          centre[0] + u * options_.sample_radius, centre[1] + v * options_.sample_radius,
          centre[2] + options_.sample_height);
        if (!pixel) {
          outside = true;
          break;
        }
        const int px = static_cast<int>(std::lround((*pixel)[0]));
        const int py = static_cast<int>(std::lround((*pixel)[1]));
        if (px < 0 || py < 0 || px >= image.width || py >= image.height) {
          outside = true;
          break;
        }
        const uint8_t * rgb = image.data + py * image.step + px * 3;
        sr += image.bgr ? rgb[2] : rgb[0];
        sg += rgb[1];
        sb += image.bgr ? rgb[0] : rgb[2];
        ++count;
      }
    }
    if (!outside && count > 0) {
      out[index] = rgb_to_lab(sr / count, sg / count, sb / count);
    }
  }
  return out;
}

void SquareClassifier::blend(Reference & ref, const Colour & colour) const
{
  if (!ref.valid) {
    ref.colour = colour;
    ref.valid = true;
    return;
  }
  for (int i = 0; i < 3; ++i) {
    ref.colour[i] += options_.learning_rate * (colour[i] - ref.colour[i]);
  }
}

void SquareClassifier::learn(
  const std::array<std::optional<Colour>, 64> & colours, const std::array<Occupancy, 64> & labels)
{
  // Pooled references are averaged over this observation's squares, then blended in.
  std::array<std::array<Colour, 3>, 2> sums{};
  std::array<std::array<int, 3>, 2> counts{};
  for (int index = 0; index < 64; ++index) {
    if (!colours[index] || labels[index] == Occupancy::kUnknown) {
      continue;
    }
    const auto cls = static_cast<int>(labels[index]);
    blend(per_square_[index][cls], *colours[index]);
    auto & sum = sums[parity(index)][cls];
    for (int i = 0; i < 3; ++i) {
      sum[i] += (*colours[index])[i];
    }
    ++counts[parity(index)][cls];
  }
  for (int p = 0; p < 2; ++p) {
    for (int cls = 0; cls < 3; ++cls) {
      if (counts[p][cls] > 0) {
        Colour mean{};
        for (int i = 0; i < 3; ++i) {
          mean[i] = sums[p][cls][i] / counts[p][cls];
        }
        blend(per_parity_[p][cls], mean);
        learned_ = true;
      }
    }
  }
}

void SquareClassifier::classify(
  const std::array<std::optional<Colour>, 64> & colours,
  std::array<Occupancy, 64> & occupancy, std::array<float, 64> & confidence) const
{
  for (int index = 0; index < 64; ++index) {
    occupancy[index] = Occupancy::kUnknown;
    confidence[index] = 0.0f;
    if (!colours[index]) {
      continue;
    }
    double best = std::numeric_limits<double>::infinity();
    double second = std::numeric_limits<double>::infinity();
    int best_cls = -1;
    for (int cls = 0; cls < 3; ++cls) {
      const Reference & own = per_square_[index][cls];
      const Reference & pooled = per_parity_[parity(index)][cls];
      const Reference * ref = own.valid ? &own : (pooled.valid ? &pooled : nullptr);
      if (!ref) {
        continue;
      }
      const double d = distance(*colours[index], ref->colour);
      if (d < best) {
        second = best;
        best = d;
        best_cls = cls;
      } else if (d < second) {
        second = d;
      }
    }
    if (best_cls < 0 || !std::isfinite(second)) {
      continue;  // fewer than two classes to choose between
    }
    occupancy[index] = static_cast<Occupancy>(best_cls);
    confidence[index] = static_cast<float>((second - best) / (second + best + 1e-9));
  }
}

std::optional<std::array<Occupancy, 64>> labels_from_fen(const std::string & fen)
{
  std::array<Occupancy, 64> labels;
  labels.fill(Occupancy::kEmpty);
  int rank = 7, file = 0;
  for (char ch : fen) {
    if (ch == ' ') {
      break;
    }
    if (ch == '/') {
      --rank;
      file = 0;
    } else if (std::isdigit(static_cast<unsigned char>(ch))) {
      file += ch - '0';
    } else {
      if (rank < 0 || file > 7) {
        return std::nullopt;
      }
      labels[file + 8 * rank] = std::isupper(static_cast<unsigned char>(ch)) ? Occupancy::kWhite : Occupancy::kBlack;
      ++file;
    }
  }
  if (rank != 0) {
    return std::nullopt;
  }
  return labels;
}

}  // namespace chessbot_perception
