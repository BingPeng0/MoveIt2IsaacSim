// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from moveit_msgs:msg/PipelineState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "moveit_msgs/msg/detail/pipeline_state__rosidl_typesupport_introspection_c.h"
#include "moveit_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "moveit_msgs/msg/detail/pipeline_state__functions.h"
#include "moveit_msgs/msg/detail/pipeline_state__struct.h"


// Include directives for member types
// Member `request`
#include "moveit_msgs/msg/motion_plan_request.h"
// Member `request`
#include "moveit_msgs/msg/detail/motion_plan_request__rosidl_typesupport_introspection_c.h"
// Member `response`
#include "moveit_msgs/msg/motion_plan_response.h"
// Member `response`
#include "moveit_msgs/msg/detail/motion_plan_response__rosidl_typesupport_introspection_c.h"
// Member `pipeline_stage`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moveit_msgs__msg__PipelineState__init(message_memory);
}

void moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_fini_function(void * message_memory)
{
  moveit_msgs__msg__PipelineState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_member_array[3] = {
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__msg__PipelineState, request),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__msg__PipelineState, response),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pipeline_stage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moveit_msgs__msg__PipelineState, pipeline_stage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_members = {
  "moveit_msgs__msg",  // message namespace
  "PipelineState",  // message name
  3,  // number of fields
  sizeof(moveit_msgs__msg__PipelineState),
  moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_member_array,  // message members
  moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_init_function,  // function to initialize message memory (memory has to be allocated)
  moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_type_support_handle = {
  0,
  &moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moveit_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, PipelineState)() {
  moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, MotionPlanRequest)();
  moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moveit_msgs, msg, MotionPlanResponse)();
  if (!moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_type_support_handle.typesupport_identifier) {
    moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &moveit_msgs__msg__PipelineState__rosidl_typesupport_introspection_c__PipelineState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
