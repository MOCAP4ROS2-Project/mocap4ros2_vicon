cmake_minimum_required(VERSION 3.5)

project(mocap_vicon_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Download and Install vicon sdk
execute_process(
	COMMAND bash "-c" "${CMAKE_CURRENT_SOURCE_DIR}/vicon_sdk_install.sh"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap_control REQUIRED)

# Used by Vicon SDK. Now we dont copy boost binaries shipped with driver.
find_package(Boost REQUIRED COMPONENTS thread date_time)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  mocap_msgs
  mocap_control
  geometry_msgs
)

include_directories(
  include
  vicon_sdk/DataStream  # For Vicon SDK
  vicon_sdk             # For StreamCommon, required by ViconSDK  
)

# Compile Vicon SDK from scratch to avoid Boost version mismatch clashes
file(GLOB_RECURSE vicon_sdk_files "${CMAKE_CURRENT_SOURCE_DIR}/vicon_sdk/**/**.cpp")
add_library(vicon_sdk ${vicon_sdk_files})
target_link_libraries(vicon_sdk PUBLIC ${Boost_LIBRARIES})

add_library(
  ${PROJECT_NAME}
src/mocap_vicon_driver/mocap_vicon_driver.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "VICON_BUILDING_LIBRARY")

add_executable(mocap_vicon_driver_main
  src/mocap_vicon_driver_main.cpp
)
ament_target_dependencies(mocap_vicon_driver_main ${dependencies})
target_link_libraries(mocap_vicon_driver_main
  ${PROJECT_NAME}
  vicon_sdk
)

ament_target_dependencies(mocap_vicon_driver ${dependencies})
target_link_libraries(mocap_vicon_driver 
  vicon_sdk
)

add_executable(ViconDataStreamSDK_CPPTest
  src/ViconDataStreamSDK_CPPTest.cpp
)
ament_target_dependencies(ViconDataStreamSDK_CPPTest ${dependencies})
target_link_libraries(ViconDataStreamSDK_CPPTest
  ${PROJECT_NAME}
  vicon_sdk
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  mocap_vicon_driver_main
  ViconDataStreamSDK_CPPTest
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME} vicon_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_mocap_vicon_driver test/test_mocap_vicon_driver.cpp)
  target_link_libraries(test_mocap_vicon_driver ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
