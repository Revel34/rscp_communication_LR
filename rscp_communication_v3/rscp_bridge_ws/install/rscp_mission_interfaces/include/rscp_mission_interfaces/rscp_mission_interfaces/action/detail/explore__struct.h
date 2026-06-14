// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rscp_mission_interfaces:action/Explore.idl
// generated code does not contain a copyright notice

#ifndef RSCP_MISSION_INTERFACES__ACTION__DETAIL__EXPLORE__STRUCT_H_
#define RSCP_MISSION_INTERFACES__ACTION__DETAIL__EXPLORE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_Goal
{
  uint8_t structure_needs_at_least_one_member;
} rscp_mission_interfaces__action__Explore_Goal;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_Goal.
typedef struct rscp_mission_interfaces__action__Explore_Goal__Sequence
{
  rscp_mission_interfaces__action__Explore_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_Result
{
  bool success;
} rscp_mission_interfaces__action__Explore_Result;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_Result.
typedef struct rscp_mission_interfaces__action__Explore_Result__Sequence
{
  rscp_mission_interfaces__action__Explore_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_Feedback
{
  double distance;
} rscp_mission_interfaces__action__Explore_Feedback;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_Feedback.
typedef struct rscp_mission_interfaces__action__Explore_Feedback__Sequence
{
  rscp_mission_interfaces__action__Explore_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rscp_mission_interfaces/action/detail/explore__struct.h"

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rscp_mission_interfaces__action__Explore_Goal goal;
} rscp_mission_interfaces__action__Explore_SendGoal_Request;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_SendGoal_Request.
typedef struct rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence
{
  rscp_mission_interfaces__action__Explore_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rscp_mission_interfaces__action__Explore_SendGoal_Response;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_SendGoal_Response.
typedef struct rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence
{
  rscp_mission_interfaces__action__Explore_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rscp_mission_interfaces__action__Explore_GetResult_Request;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_GetResult_Request.
typedef struct rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence
{
  rscp_mission_interfaces__action__Explore_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.h"

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_GetResult_Response
{
  int8_t status;
  rscp_mission_interfaces__action__Explore_Result result;
} rscp_mission_interfaces__action__Explore_GetResult_Response;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_GetResult_Response.
typedef struct rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence
{
  rscp_mission_interfaces__action__Explore_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.h"

/// Struct defined in action/Explore in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__Explore_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rscp_mission_interfaces__action__Explore_Feedback feedback;
} rscp_mission_interfaces__action__Explore_FeedbackMessage;

// Struct for a sequence of rscp_mission_interfaces__action__Explore_FeedbackMessage.
typedef struct rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence
{
  rscp_mission_interfaces__action__Explore_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__Explore_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RSCP_MISSION_INTERFACES__ACTION__DETAIL__EXPLORE__STRUCT_H_
