// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rscp_mission_interfaces:action/NavigateToGPS.idl
// generated code does not contain a copyright notice

#ifndef RSCP_MISSION_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_GPS__STRUCT_H_
#define RSCP_MISSION_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_GPS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_Goal
{
  double lat;
  double lon;
} rscp_mission_interfaces__action__NavigateToGPS_Goal;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_Goal.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_Result
{
  bool success;
} rscp_mission_interfaces__action__NavigateToGPS_Result;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_Result.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_Feedback
{
  double distance_remaining;
} rscp_mission_interfaces__action__NavigateToGPS_Feedback;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_Feedback.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rscp_mission_interfaces/action/detail/navigate_to_gps__struct.h"

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rscp_mission_interfaces__action__NavigateToGPS_Goal goal;
} rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rscp_mission_interfaces/action/detail/navigate_to_gps__struct.h"

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response
{
  int8_t status;
  rscp_mission_interfaces__action__NavigateToGPS_Result result;
} rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rscp_mission_interfaces/action/detail/navigate_to_gps__struct.h"

/// Struct defined in action/NavigateToGPS in the package rscp_mission_interfaces.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rscp_mission_interfaces__action__NavigateToGPS_Feedback feedback;
} rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage;

// Struct for a sequence of rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage.
typedef struct rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence
{
  rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rscp_mission_interfaces__action__NavigateToGPS_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RSCP_MISSION_INTERFACES__ACTION__DETAIL__NAVIGATE_TO_GPS__STRUCT_H_
