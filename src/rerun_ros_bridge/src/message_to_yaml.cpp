// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0

#include "rerun_ros_bridge/message_to_yaml.hpp"

#include <cstdlib>
#include <iomanip>
#include <new>
#include <sstream>
#include <stdexcept>

#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rmw/rmw.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace rerun_ros_bridge
{

namespace ts = rosidl_typesupport_introspection_cpp;

struct MessageToYaml::Impl
{
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * cpp_typesupport{nullptr};
  const ts::MessageMembers * members{nullptr};
};

namespace
{

void render_scalar(uint8_t type_id, const void * p, std::ostream & out)
{
  switch (type_id) {
    case ts::ROS_TYPE_FLOAT: out << *static_cast<const float *>(p); break;
    case ts::ROS_TYPE_DOUBLE: out << *static_cast<const double *>(p); break;
    case ts::ROS_TYPE_LONG_DOUBLE: out << *static_cast<const long double *>(p); break;
    case ts::ROS_TYPE_CHAR: out << static_cast<int>(*static_cast<const char *>(p)); break;
    case ts::ROS_TYPE_BOOLEAN: out << (*static_cast<const bool *>(p) ? "true" : "false"); break;
    case ts::ROS_TYPE_OCTET:
    case ts::ROS_TYPE_UINT8: out << static_cast<int>(*static_cast<const uint8_t *>(p)); break;
    case ts::ROS_TYPE_INT8: out << static_cast<int>(*static_cast<const int8_t *>(p)); break;
    case ts::ROS_TYPE_UINT16: out << *static_cast<const uint16_t *>(p); break;
    case ts::ROS_TYPE_INT16: out << *static_cast<const int16_t *>(p); break;
    case ts::ROS_TYPE_UINT32: out << *static_cast<const uint32_t *>(p); break;
    case ts::ROS_TYPE_INT32: out << *static_cast<const int32_t *>(p); break;
    case ts::ROS_TYPE_UINT64: out << *static_cast<const uint64_t *>(p); break;
    case ts::ROS_TYPE_INT64: out << *static_cast<const int64_t *>(p); break;
    case ts::ROS_TYPE_STRING: out << std::quoted(*static_cast<const std::string *>(p)); break;
    default: out << "<unsupported>"; break;
  }
}

void render_message(
  const ts::MessageMembers * members, const void * data, std::ostream & out, int indent, size_t max_items);

void render_value(
  const ts::MessageMember & field, const void * p, std::ostream & out, int indent, size_t max_items)
{
  if (field.type_id_ == ts::ROS_TYPE_MESSAGE) {
    out << "\n";
    render_message(static_cast<const ts::MessageMembers *>(field.members_->data), p, out, indent + 2, max_items);
  } else {
    render_scalar(field.type_id_, p, out);
    out << "\n";
  }
}

void render_message(
  const ts::MessageMembers * members, const void * data, std::ostream & out, int indent, size_t max_items)
{
  const std::string pad(indent, ' ');
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const ts::MessageMember & field = members->members_[i];
    const void * p = static_cast<const uint8_t *>(data) + field.offset_;
    out << pad << field.name_ << ":";
    if (!field.is_array_) {
      out << (field.type_id_ == ts::ROS_TYPE_MESSAGE ? "" : " ");
      render_value(field, p, out, indent, max_items);
      continue;
    }
    const size_t count = field.size_function ? field.size_function(p) : field.array_size_;
    if (count == 0) {
      out << " []\n";
      continue;
    }
    out << "\n";
    const size_t shown = std::min(count, max_items);
    for (size_t j = 0; j < shown; ++j) {
      out << pad << "- ";
      const void * element = field.get_const_function(p, j);
      if (field.type_id_ == ts::ROS_TYPE_MESSAGE) {
        std::ostringstream nested;
        render_message(static_cast<const ts::MessageMembers *>(field.members_->data), element, nested, indent + 2,
          max_items);
        out << "\n" << nested.str();
      } else {
        render_scalar(field.type_id_, element, out);
        out << "\n";
      }
    }
    if (shown < count) {
      out << pad << "- ... (" << count - shown << " more)\n";
    }
  }
}

}  // namespace

MessageToYaml::MessageToYaml(const std::string & type)
: impl_(std::make_unique<Impl>())
{
  impl_->cpp_library = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
  impl_->cpp_typesupport = rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_cpp",
      *impl_->cpp_library);
  impl_->introspection_library = rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
  const auto * introspection = rclcpp::get_message_typesupport_handle(type,
      "rosidl_typesupport_introspection_cpp", *impl_->introspection_library);
  impl_->members = static_cast<const ts::MessageMembers *>(introspection->data);
  if (impl_->members == nullptr) {
    throw std::runtime_error("no introspection data for " + type);
  }
}

MessageToYaml::~MessageToYaml() = default;

std::string MessageToYaml::render(const rclcpp::SerializedMessage & msg, size_t max_array_items) const
{
  const auto * members = impl_->members;
  const std::align_val_t alignment{alignof(std::max_align_t)};
  void * storage = ::operator new(members->size_of_, alignment);
  members->init_function(storage, rosidl_runtime_cpp::MessageInitialization::ALL);

  std::ostringstream out;
  const rmw_ret_t ret = rmw_deserialize(&msg.get_rcl_serialized_message(), impl_->cpp_typesupport, storage);
  if (ret == RMW_RET_OK) {
    render_message(members, storage, out, 0, max_array_items);
  } else {
    out << "<failed to deserialize>";
  }

  members->fini_function(storage);
  ::operator delete(storage, alignment);
  return out.str();
}

}  // namespace rerun_ros_bridge
