// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#ifndef CHESSBOT_PERCEPTION__KV_CLIENT_HPP_
#define CHESSBOT_PERCEPTION__KV_CLIENT_HPP_

#include <memory>
#include <optional>
#include <string>

namespace chessbot_perception
{

/// Reads values from the Zenoh router's key-value store (e.g. the calibration profile).
class KvClient
{
public:
  /// endpoint: the router, e.g. "tcp/127.0.0.1:7447".
  explicit KvClient(const std::string & endpoint);
  ~KvClient();
  KvClient(const KvClient &) = delete;
  KvClient & operator=(const KvClient &) = delete;

  bool connected() const;

  /// The value stored under key, as a string; nothing if absent or unreachable.
  std::optional<std::string> get(const std::string & key, int timeout_ms = 2000) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chessbot_perception

#endif  // CHESSBOT_PERCEPTION__KV_CLIENT_HPP_
