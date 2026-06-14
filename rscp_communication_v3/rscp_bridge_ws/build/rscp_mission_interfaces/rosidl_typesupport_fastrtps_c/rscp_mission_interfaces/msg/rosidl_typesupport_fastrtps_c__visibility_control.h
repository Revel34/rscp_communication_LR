// generated from
// rosidl_typesupport_fastrtps_c/resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef RSCP_MISSION_INTERFACES__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
#define RSCP_MISSION_INTERFACES__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_rscp_mission_interfaces __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rscp_mission_interfaces __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_rscp_mission_interfaces __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rscp_mission_interfaces __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_C_BUILDING_DLL_rscp_mission_interfaces
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rscp_mission_interfaces ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_rscp_mission_interfaces
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rscp_mission_interfaces ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rscp_mission_interfaces
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_rscp_mission_interfaces __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rscp_mission_interfaces
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rscp_mission_interfaces __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rscp_mission_interfaces
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // RSCP_MISSION_INTERFACES__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
