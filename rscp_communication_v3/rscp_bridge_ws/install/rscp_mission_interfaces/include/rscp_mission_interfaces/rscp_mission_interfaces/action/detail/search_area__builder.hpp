// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rscp_mission_interfaces:action/SearchArea.idl
// generated code does not contain a copyright notice

#ifndef RSCP_MISSION_INTERFACES__ACTION__DETAIL__SEARCH_AREA__BUILDER_HPP_
#define RSCP_MISSION_INTERFACES__ACTION__DETAIL__SEARCH_AREA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rscp_mission_interfaces/action/detail/search_area__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_Goal_radius
{
public:
  explicit Init_SearchArea_Goal_radius(::rscp_mission_interfaces::action::SearchArea_Goal & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_Goal radius(::rscp_mission_interfaces::action::SearchArea_Goal::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Goal msg_;
};

class Init_SearchArea_Goal_lon
{
public:
  explicit Init_SearchArea_Goal_lon(::rscp_mission_interfaces::action::SearchArea_Goal & msg)
  : msg_(msg)
  {}
  Init_SearchArea_Goal_radius lon(::rscp_mission_interfaces::action::SearchArea_Goal::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_SearchArea_Goal_radius(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Goal msg_;
};

class Init_SearchArea_Goal_lat
{
public:
  Init_SearchArea_Goal_lat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_Goal_lon lat(::rscp_mission_interfaces::action::SearchArea_Goal::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_SearchArea_Goal_lon(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_Goal>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_Goal_lat();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_Result_success
{
public:
  explicit Init_SearchArea_Result_success(::rscp_mission_interfaces::action::SearchArea_Result & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_Result success(::rscp_mission_interfaces::action::SearchArea_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Result msg_;
};

class Init_SearchArea_Result_found_lon
{
public:
  explicit Init_SearchArea_Result_found_lon(::rscp_mission_interfaces::action::SearchArea_Result & msg)
  : msg_(msg)
  {}
  Init_SearchArea_Result_success found_lon(::rscp_mission_interfaces::action::SearchArea_Result::_found_lon_type arg)
  {
    msg_.found_lon = std::move(arg);
    return Init_SearchArea_Result_success(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Result msg_;
};

class Init_SearchArea_Result_found_lat
{
public:
  Init_SearchArea_Result_found_lat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_Result_found_lon found_lat(::rscp_mission_interfaces::action::SearchArea_Result::_found_lat_type arg)
  {
    msg_.found_lat = std::move(arg);
    return Init_SearchArea_Result_found_lon(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_Result>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_Result_found_lat();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_Feedback_current_lon
{
public:
  explicit Init_SearchArea_Feedback_current_lon(::rscp_mission_interfaces::action::SearchArea_Feedback & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_Feedback current_lon(::rscp_mission_interfaces::action::SearchArea_Feedback::_current_lon_type arg)
  {
    msg_.current_lon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Feedback msg_;
};

class Init_SearchArea_Feedback_current_lat
{
public:
  Init_SearchArea_Feedback_current_lat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_Feedback_current_lon current_lat(::rscp_mission_interfaces::action::SearchArea_Feedback::_current_lat_type arg)
  {
    msg_.current_lat = std::move(arg);
    return Init_SearchArea_Feedback_current_lon(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_Feedback>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_Feedback_current_lat();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_SendGoal_Request_goal
{
public:
  explicit Init_SearchArea_SendGoal_Request_goal(::rscp_mission_interfaces::action::SearchArea_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_SendGoal_Request goal(::rscp_mission_interfaces::action::SearchArea_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_SendGoal_Request msg_;
};

class Init_SearchArea_SendGoal_Request_goal_id
{
public:
  Init_SearchArea_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_SendGoal_Request_goal goal_id(::rscp_mission_interfaces::action::SearchArea_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SearchArea_SendGoal_Request_goal(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_SendGoal_Request>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_SendGoal_Request_goal_id();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_SendGoal_Response_stamp
{
public:
  explicit Init_SearchArea_SendGoal_Response_stamp(::rscp_mission_interfaces::action::SearchArea_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_SendGoal_Response stamp(::rscp_mission_interfaces::action::SearchArea_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_SendGoal_Response msg_;
};

class Init_SearchArea_SendGoal_Response_accepted
{
public:
  Init_SearchArea_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_SendGoal_Response_stamp accepted(::rscp_mission_interfaces::action::SearchArea_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SearchArea_SendGoal_Response_stamp(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_SendGoal_Response>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_SendGoal_Response_accepted();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_GetResult_Request_goal_id
{
public:
  Init_SearchArea_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rscp_mission_interfaces::action::SearchArea_GetResult_Request goal_id(::rscp_mission_interfaces::action::SearchArea_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_GetResult_Request>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_GetResult_Request_goal_id();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_GetResult_Response_result
{
public:
  explicit Init_SearchArea_GetResult_Response_result(::rscp_mission_interfaces::action::SearchArea_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_GetResult_Response result(::rscp_mission_interfaces::action::SearchArea_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_GetResult_Response msg_;
};

class Init_SearchArea_GetResult_Response_status
{
public:
  Init_SearchArea_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_GetResult_Response_result status(::rscp_mission_interfaces::action::SearchArea_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SearchArea_GetResult_Response_result(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_GetResult_Response>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_GetResult_Response_status();
}

}  // namespace rscp_mission_interfaces


namespace rscp_mission_interfaces
{

namespace action
{

namespace builder
{

class Init_SearchArea_FeedbackMessage_feedback
{
public:
  explicit Init_SearchArea_FeedbackMessage_feedback(::rscp_mission_interfaces::action::SearchArea_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::rscp_mission_interfaces::action::SearchArea_FeedbackMessage feedback(::rscp_mission_interfaces::action::SearchArea_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_FeedbackMessage msg_;
};

class Init_SearchArea_FeedbackMessage_goal_id
{
public:
  Init_SearchArea_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SearchArea_FeedbackMessage_feedback goal_id(::rscp_mission_interfaces::action::SearchArea_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SearchArea_FeedbackMessage_feedback(msg_);
  }

private:
  ::rscp_mission_interfaces::action::SearchArea_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rscp_mission_interfaces::action::SearchArea_FeedbackMessage>()
{
  return rscp_mission_interfaces::action::builder::Init_SearchArea_FeedbackMessage_goal_id();
}

}  // namespace rscp_mission_interfaces

#endif  // RSCP_MISSION_INTERFACES__ACTION__DETAIL__SEARCH_AREA__BUILDER_HPP_
