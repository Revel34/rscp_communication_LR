// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from rscp_mission_interfaces:action/Explore.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_Goal_type_support_ids_t;

static const _Explore_Goal_type_support_ids_t _Explore_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_Goal_type_support_symbol_names_t _Explore_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_Goal)),
  }
};

typedef struct _Explore_Goal_type_support_data_t
{
  void * data[2];
} _Explore_Goal_type_support_data_t;

static _Explore_Goal_type_support_data_t _Explore_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_Goal_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_Goal>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_Goal)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_Result_type_support_ids_t;

static const _Explore_Result_type_support_ids_t _Explore_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_Result_type_support_symbol_names_t _Explore_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_Result)),
  }
};

typedef struct _Explore_Result_type_support_data_t
{
  void * data[2];
} _Explore_Result_type_support_data_t;

static _Explore_Result_type_support_data_t _Explore_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_Result_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_Result>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_Result)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_Feedback_type_support_ids_t;

static const _Explore_Feedback_type_support_ids_t _Explore_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_Feedback_type_support_symbol_names_t _Explore_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_Feedback)),
  }
};

typedef struct _Explore_Feedback_type_support_data_t
{
  void * data[2];
} _Explore_Feedback_type_support_data_t;

static _Explore_Feedback_type_support_data_t _Explore_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_Feedback_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_Feedback>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_Feedback)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_SendGoal_Request_type_support_ids_t;

static const _Explore_SendGoal_Request_type_support_ids_t _Explore_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_SendGoal_Request_type_support_symbol_names_t _Explore_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_SendGoal_Request)),
  }
};

typedef struct _Explore_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Explore_SendGoal_Request_type_support_data_t;

static _Explore_SendGoal_Request_type_support_data_t _Explore_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_SendGoal_Request_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_SendGoal_Request>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_SendGoal_Request)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_SendGoal_Response_type_support_ids_t;

static const _Explore_SendGoal_Response_type_support_ids_t _Explore_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_SendGoal_Response_type_support_symbol_names_t _Explore_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_SendGoal_Response)),
  }
};

typedef struct _Explore_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Explore_SendGoal_Response_type_support_data_t;

static _Explore_SendGoal_Response_type_support_data_t _Explore_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_SendGoal_Response_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_SendGoal_Response>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_SendGoal_Response)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_SendGoal_type_support_ids_t;

static const _Explore_SendGoal_type_support_ids_t _Explore_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_SendGoal_type_support_symbol_names_t _Explore_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_SendGoal)),
  }
};

typedef struct _Explore_SendGoal_type_support_data_t
{
  void * data[2];
} _Explore_SendGoal_type_support_data_t;

static _Explore_SendGoal_type_support_data_t _Explore_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_SendGoal_service_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Explore_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Explore_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Explore_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rscp_mission_interfaces::action::Explore_SendGoal>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<rscp_mission_interfaces::action::Explore_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_GetResult_Request_type_support_ids_t;

static const _Explore_GetResult_Request_type_support_ids_t _Explore_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_GetResult_Request_type_support_symbol_names_t _Explore_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_GetResult_Request)),
  }
};

typedef struct _Explore_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Explore_GetResult_Request_type_support_data_t;

static _Explore_GetResult_Request_type_support_data_t _Explore_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_GetResult_Request_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_GetResult_Request>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_GetResult_Request)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_GetResult_Response_type_support_ids_t;

static const _Explore_GetResult_Response_type_support_ids_t _Explore_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_GetResult_Response_type_support_symbol_names_t _Explore_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_GetResult_Response)),
  }
};

typedef struct _Explore_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Explore_GetResult_Response_type_support_data_t;

static _Explore_GetResult_Response_type_support_data_t _Explore_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_GetResult_Response_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_GetResult_Response>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_GetResult_Response)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_GetResult_type_support_ids_t;

static const _Explore_GetResult_type_support_ids_t _Explore_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_GetResult_type_support_symbol_names_t _Explore_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_GetResult)),
  }
};

typedef struct _Explore_GetResult_type_support_data_t
{
  void * data[2];
} _Explore_GetResult_type_support_data_t;

static _Explore_GetResult_type_support_data_t _Explore_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_GetResult_service_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Explore_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Explore_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Explore_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rscp_mission_interfaces::action::Explore_GetResult>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<rscp_mission_interfaces::action::Explore_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Explore_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Explore_FeedbackMessage_type_support_ids_t;

static const _Explore_FeedbackMessage_type_support_ids_t _Explore_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Explore_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Explore_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Explore_FeedbackMessage_type_support_symbol_names_t _Explore_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rscp_mission_interfaces, action, Explore_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rscp_mission_interfaces, action, Explore_FeedbackMessage)),
  }
};

typedef struct _Explore_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Explore_FeedbackMessage_type_support_data_t;

static _Explore_FeedbackMessage_type_support_data_t _Explore_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Explore_FeedbackMessage_message_typesupport_map = {
  2,
  "rscp_mission_interfaces",
  &_Explore_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Explore_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Explore_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Explore_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Explore_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rscp_mission_interfaces::action::Explore_FeedbackMessage>()
{
  return &::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore_FeedbackMessage)() {
  return get_message_type_support_handle<rscp_mission_interfaces::action::Explore_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "rscp_mission_interfaces/action/detail/explore__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace rscp_mission_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t Explore_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace rscp_mission_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<rscp_mission_interfaces::action::Explore>()
{
  using ::rscp_mission_interfaces::action::rosidl_typesupport_cpp::Explore_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  Explore_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::rscp_mission_interfaces::action::Explore::Impl::SendGoalService>();
  Explore_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::rscp_mission_interfaces::action::Explore::Impl::GetResultService>();
  Explore_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::rscp_mission_interfaces::action::Explore::Impl::CancelGoalService>();
  Explore_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::rscp_mission_interfaces::action::Explore::Impl::FeedbackMessage>();
  Explore_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::rscp_mission_interfaces::action::Explore::Impl::GoalStatusMessage>();
  return &Explore_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, rscp_mission_interfaces, action, Explore)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<rscp_mission_interfaces::action::Explore>();
}

#ifdef __cplusplus
}
#endif
