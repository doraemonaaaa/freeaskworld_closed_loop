# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target simulator_messages::simulator_messages
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${simulator_messages_TARGETS}.
if(simulator_messages_TARGETS AND NOT TARGET simulator_messages::simulator_messages)
  add_library(simulator_messages::simulator_messages INTERFACE IMPORTED)
  set_target_properties(simulator_messages::simulator_messages PROPERTIES
    INTERFACE_LINK_LIBRARIES "${simulator_messages_TARGETS}")
endif()
