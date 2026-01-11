// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from simulator_messages:msg/SimulatorCommand.idl
// generated code does not contain a copyright notice

#include "simulator_messages/msg/detail/simulator_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
const rosidl_type_hash_t *
simulator_messages__msg__SimulatorCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0xd0, 0x93, 0xdb, 0xa4, 0xe2, 0x8c, 0x5f,
      0x11, 0x00, 0xd0, 0x01, 0x10, 0x1d, 0x41, 0xef,
      0x2d, 0x0a, 0xbd, 0x78, 0x0b, 0xb3, 0x06, 0x57,
      0x77, 0xec, 0x1e, 0xc1, 0xa1, 0xc4, 0xd1, 0x38,
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

static char simulator_messages__msg__SimulatorCommand__TYPE_NAME[] = "simulator_messages/msg/SimulatorCommand";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char simulator_messages__msg__SimulatorCommand__FIELD_NAME__header[] = "header";
static char simulator_messages__msg__SimulatorCommand__FIELD_NAME__method[] = "method";
static char simulator_messages__msg__SimulatorCommand__FIELD_NAME__method_params[] = "method_params";

static rosidl_runtime_c__type_description__Field simulator_messages__msg__SimulatorCommand__FIELDS[] = {
  {
    {simulator_messages__msg__SimulatorCommand__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {simulator_messages__msg__SimulatorCommand__FIELD_NAME__method, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {simulator_messages__msg__SimulatorCommand__FIELD_NAME__method_params, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription simulator_messages__msg__SimulatorCommand__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
simulator_messages__msg__SimulatorCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {simulator_messages__msg__SimulatorCommand__TYPE_NAME, 39, 39},
      {simulator_messages__msg__SimulatorCommand__FIELDS, 3, 3},
    },
    {simulator_messages__msg__SimulatorCommand__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "# \\xe8\\xb0\\x83\\xe7\\x94\\xa8\\xe6\\x8e\\xa5\\xe6\\x94\\xb6\\xe7\\xab\\xaf\\xe7\\x9a\\x84\\xe5\\x87\\xbd\\xe6\\x95\\xb0\\xe6\\x96\\xb9\\xe6\\xb3\\x95\\xe5\\x90\\x8d\n"
  "string method\n"
  "\n"
  "string method_params";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
simulator_messages__msg__SimulatorCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {simulator_messages__msg__SimulatorCommand__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 82, 82},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
simulator_messages__msg__SimulatorCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *simulator_messages__msg__SimulatorCommand__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
