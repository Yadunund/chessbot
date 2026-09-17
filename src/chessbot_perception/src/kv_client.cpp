// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include "chessbot_perception/kv_client.hpp"

#include <zenoh.h>

namespace chessbot_perception
{

struct KvClient::Impl
{
  z_owned_session_t session;
  bool open{false};
};

KvClient::KvClient(const std::string & endpoint)
: impl_(std::make_unique<Impl>())
{
  z_owned_config_t config;
  if (z_config_default(&config) != Z_OK) {
    return;
  }
  const std::string endpoints = "[\"" + endpoint + "\"]";
  zc_config_insert_json5(z_loan_mut(config), "mode", "\"client\"");
  zc_config_insert_json5(z_loan_mut(config), "connect/endpoints", endpoints.c_str());
  zc_config_insert_json5(z_loan_mut(config), "scouting/multicast/enabled", "false");
  impl_->open = z_open(&impl_->session, z_move(config), nullptr) == Z_OK;
}

KvClient::~KvClient()
{
  if (impl_->open) {
    z_drop(z_move(impl_->session));
  }
}

bool KvClient::connected() const
{
  return impl_->open;
}

std::optional<std::string> KvClient::get(const std::string & key, int timeout_ms) const
{
  if (!impl_->open) {
    return std::nullopt;
  }
  z_view_keyexpr_t key_expr;
  std::string key_copy = key;  // z_view_keyexpr_from_str may canonise in place
  if (z_view_keyexpr_from_str(&key_expr, key_copy.data()) != Z_OK) {
    return std::nullopt;
  }
  z_owned_fifo_handler_reply_t handler;
  z_owned_closure_reply_t closure;
  z_fifo_channel_reply_new(&closure, &handler, 16);
  z_get_options_t options;
  z_get_options_default(&options);
  options.timeout_ms = static_cast<uint64_t>(timeout_ms);
  if (z_get(z_loan(impl_->session), z_loan(key_expr), "", z_move(closure), &options) != Z_OK) {
    z_drop(z_move(handler));
    return std::nullopt;
  }
  std::optional<std::string> value;
  z_owned_reply_t reply;
  // Blocks until a reply arrives or the query times out and the channel closes.
  while (z_recv(z_loan(handler), &reply) == Z_OK) {
    if (!value && z_reply_is_ok(z_loan(reply))) {
      const z_loaned_sample_t * sample = z_reply_ok(z_loan(reply));
      z_owned_string_t text;
      if (z_bytes_to_string(z_sample_payload(sample), &text) == Z_OK) {
        value = std::string(z_string_data(z_loan(text)), z_string_len(z_loan(text)));
        z_drop(z_move(text));
      }
    }
    z_drop(z_move(reply));
  }
  z_drop(z_move(handler));
  return value;
}

}  // namespace chessbot_perception
