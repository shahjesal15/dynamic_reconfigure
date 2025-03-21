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
find_package(fmt REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(
  include/dynamic_reconfigure
)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/dynamic_reconfigure/rviz/dynamic_reconfigure.hpp
  include/dynamic_reconfigure/rviz/components/logger.hpp
)

add_library(watchdog_timer src/dependencies/watchdog_timer.cpp)
set_target_properties(watchdog_timer PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(service_wrapper src/dependencies/service_wrapper.cpp)
set_target_properties(service_wrapper PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(service_wrapper rclcpp rcl_interfaces std_msgs)
target_link_libraries(service_wrapper fmt::fmt watchdog_timer)

add_library(logger src/rviz/components/logger.cpp ${MOC_FILES})
set_target_properties(logger PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(logger Qt5::Widgets fmt::fmt Qt5::Core)

add_executable(service_wrapper_test tests/service_wrapper_test.cpp)
ament_target_dependencies(service_wrapper_test rclcpp rcl_interfaces std_msgs)
target_link_libraries(service_wrapper_test service_wrapper)


add_library(dynamic_reconfigure SHARED src/rviz/dynamic_reconfigure.cpp ${MOC_FILES})
target_link_libraries(dynamic_reconfigure service_wrapper logger)

target_include_directories(dynamic_reconfigure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamic_reconfigure
  pluginlib
  rclcpp
  std_msgs
  rviz_common
  Qt5
)

install(
    TARGETS 
      service_wrapper_test
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  dynamic_reconfigure service_wrapper logger watchdog_timer
  EXPORT dynamic_reconfigure
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
    EXPORT dynamic_reconfigure
    FILE dynamic_reconfigure-targets.cmake
    NAMESPACE dynamic_reconfigure::
    DESTINATION share/dynamic_reconfigure/cmake
)

install(FILES
  dynamic_reconfigure.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(dynamic_reconfigure)
pluginlib_export_plugin_description_file(rviz_common dynamic_reconfigure.xml)


ament_package()