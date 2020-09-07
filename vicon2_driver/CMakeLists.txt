cmake_minimum_required(VERSION 3.5)

project(vicon2_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4ros2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap4ros2_msgs
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED src/vicon2_driver.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(vicon2_driver_main
  src/vicon2_driver_main.cpp
)
ament_target_dependencies(vicon2_driver_main ${dependencies})
target_link_libraries(vicon2_driver_main
  ${PROJECT_NAME}
  ViconDataStreamSDK_CPP
)

ament_target_dependencies(vicon2_driver ${dependencies})
target_link_libraries(vicon2_driver ViconDataStreamSDK_CPP)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  vicon2_driver_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_vicon2_driver test/test_vicon2_driver.cpp)
  target_link_libraries(test_vicon2_driver ${PROJECT_NAME})

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
