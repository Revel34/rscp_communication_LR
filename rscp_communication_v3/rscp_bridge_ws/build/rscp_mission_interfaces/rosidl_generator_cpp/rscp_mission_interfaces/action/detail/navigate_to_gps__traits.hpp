// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rscp_mission_interfaces:action/NavigateToGPS.idl
// generated code does not contain a copyright notice

#ifndef RSCP_MISSION_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_GPS__TRAITS_HPP_
#define RSCP_MISSION_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_GPS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rscp_mission_interfaces/action/detail/navigate_to_gps__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_Goal & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_Goal>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_Goal";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_Goal>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_Goal";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_Result & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_Result>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_Result";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_Result>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_Result";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: distance_remaining
  {
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: distance_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_Feedback & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_Feedback>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_Feedback";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_Feedback>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_Feedback";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "rscp_mission_interfaces/action/detail/navigate_to_gps__traits.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_SendGoal_Request";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_SendGoal_Response";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_SendGoal>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_SendGoal";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_SendGoal>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_SendGoal";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>::value &&
    has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>::value &&
    has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<rscp_mission_interfaces::action::NavigateToGPS_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rscp_mission_interfaces::action::NavigateToGPS_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_GetResult_Request";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "rscp_mission_interfaces/action/detail/navigate_to_gps__traits.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_GetResult_Response";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_Result>::value> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_Result>::value> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_GetResult>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_GetResult";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_GetResult>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_GetResult";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>::value &&
    has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>::value &&
    has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>::value
  >
{
};

template<>
struct is_service<rscp_mission_interfaces::action::NavigateToGPS_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rscp_mission_interfaces::action::NavigateToGPS_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "rscp_mission_interfaces/action/detail/navigate_to_gps__traits.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToGPS_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToGPS_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToGPS_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rscp_mission_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  rscp_mission_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rscp_mission_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage & msg)
{
  return rscp_mission_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage>()
{
  return "rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage";
}

template<>
inline const char * name<rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage>()
{
  return "rscp_mission_interfaces/action/NavigateToGPS_FeedbackMessage";
}

template<>
struct has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<rscp_mission_interfaces::action::NavigateToGPS_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<rscp_mission_interfaces::action::NavigateToGPS_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rscp_mission_interfaces::action::NavigateToGPS_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<rscp_mission_interfaces::action::NavigateToGPS>
  : std::true_type
{
};

template<>
struct is_action_goal<rscp_mission_interfaces::action::NavigateToGPS_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<rscp_mission_interfaces::action::NavigateToGPS_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<rscp_mission_interfaces::action::NavigateToGPS_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // RSCP_MISSION_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_GPS__TRAITS_HPP_
