// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

#include "simulator_messages/msg/detail/navigation_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
const rosidl_type_hash_t *
simulator_messages__msg__NavigationCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0x60, 0xb9, 0x45, 0xa0, 0x37, 0x5f, 0xd8,
      0xe3, 0x81, 0x63, 0x4c, 0x54, 0xf3, 0xab, 0x7b,
      0xa6, 0xb5, 0x61, 0x3a, 0xa2, 0xfd, 0x17, 0x46,
      0x42, 0xaf, 0xda, 0x04, 0x73, 0xec, 0x7f, 0xc7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char simulator_messages__msg__NavigationCommand__TYPE_NAME[] = "simulator_messages/msg/NavigationCommand";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char simulator_messages__msg__NavigationCommand__FIELD_NAME__header[] = "header";
static char simulator_messages__msg__NavigationCommand__FIELD_NAME__local_position_offset[] = "local_position_offset";
static char simulator_messages__msg__NavigationCommand__FIELD_NAME__local_rotation_offset[] = "local_rotation_offset";
static char simulator_messages__msg__NavigationCommand__FIELD_NAME__is_stop[] = "is_stop";

static rosidl_runtime_c__type_description__Field simulator_messages__msg__NavigationCommand__FIELDS[] = {
  {
    {simulator_messages__msg__NavigationCommand__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {simulator_messages__msg__NavigationCommand__FIELD_NAME__local_position_offset, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {simulator_messages__msg__NavigationCommand__FIELD_NAME__local_rotation_offset, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {simulator_messages__msg__NavigationCommand__FIELD_NAME__is_stop, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription simulator_messages__msg__NavigationCommand__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
simulator_messages__msg__NavigationCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {simulator_messages__msg__NavigationCommand__TYPE_NAME, 40, 40},
      {simulator_messages__msg__NavigationCommand__FIELDS, 4, 4},
    },
    {simulator_messages__msg__NavigationCommand__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Header\n"
  "std_msgs/Header header\n"
  "\n"
  "# \\xe4\\xbd\\x8d\\xe7\\xa7\\xbb\\xe5\\x81\\x8f\\xe7\\xa7\\xbb\\xef\\xbc\\x88\\xe5\\xb1\\x80\\xe9\\x83\\xa8\\xe5\\x9d\\x90\\xe6\\xa0\\x87\\xef\\xbc\\x89\n"
  "float64[3] local_position_offset\n"
  "\n"
  "# \\xe5\\x9b\\x9b\\xe5\\x85\\x83\\xe6\\x95\\xb0\\xe6\\x97\\x8b\\xe8\\xbd\\xac\n"
  "float64[4] local_rotation_offset\n"
  "\n"
  "# \\xe6\\x98\\xaf\\xe5\\x90\\xa6\\xe5\\x81\\x9c\\xe6\\xad\\xa2\n"
  "bool is_stop";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
simulator_messages__msg__NavigationCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {simulator_messages__msg__NavigationCommand__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 142, 142},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
simulator_messages__msg__NavigationCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *simulator_messages__msg__NavigationCommand__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
