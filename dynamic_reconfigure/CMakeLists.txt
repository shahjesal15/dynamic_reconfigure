cmake_minimum_required(VERSION 3.8)
project(dynamic_reconfigure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include/dynamic_reconfigure
)

add_library(service_wrapper src/dependencies/service_wrapper.cpp)
ament_target_dependencies(service_wrapper rclcpp rcl_interfaces std_msgs)

add_executable(service_wrapper_test tests/service_wrapper_test.cpp)
ament_target_dependencies(service_wrapper_test rclcpp rcl_interfaces)
target_link_libraries(service_wrapper_test service_wrapper)


install(TARGETS
    service_wrapper_test
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()