cmake_minimum_required(VERSION 3.5)

project(mocap4r2_vicon_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mocap_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

# Used by Vicon SDK. Now we dont copy boost binaries shipped with driver.
find_package(Boost REQUIRED COMPONENTS thread date_time chrono)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  mocap_interfaces
  mocap4r2_control
  geometry_msgs
)

file(GLOB_RECURSE VICON_HEADER_FILES "ThirdParty/*.h")
file(GLOB_RECURSE VICON_SOURCES "ThirdParty/*.cpp")

set(VICON_INCLUDE_DIRS "")
foreach(HEADER ${VICON_HEADER_FILES})
    get_filename_component(DIR ${HEADER} DIRECTORY)
    set(VICON_INCLUDE_DIRS ${VICON_INCLUDE_DIRS} ${DIR})
endforeach()
list(REMOVE_DUPLICATES VICON_INCLUDE_DIRS)


include_directories(
  include
  ThirdParty/DataStream
  ThirdParty
  ${VICON_INCLUDE_DIRS}
)

add_library(vicon_sdk SHARED ${VICON_SOURCES})
target_link_libraries(vicon_sdk PUBLIC ${Boost_LIBRARIES})


add_library(
  ${PROJECT_NAME}
src/mocap4r2_vicon_driver/mocap4r2_vicon_driver.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "VICON_BUILDING_LIBRARY")

add_executable(mocap4r2_vicon_driver_main
  src/mocap4r2_vicon_driver_main.cpp
)
ament_target_dependencies(mocap4r2_vicon_driver_main ${dependencies})
target_link_libraries(mocap4r2_vicon_driver_main
  ${PROJECT_NAME}
  vicon_sdk
)

ament_target_dependencies(mocap4r2_vicon_driver ${dependencies})
target_link_libraries(mocap4r2_vicon_driver
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
  mocap4r2_vicon_driver_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME} vicon_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)


  file(GLOB_RECURSE EXCLUDE_FILES ThirdParty/*)

  ament_copyright(EXCLUDE ${EXCLUDE_FILES})
  ament_cppcheck(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )
  ament_cpplint(EXCLUDE ${EXCLUDE_FILES})
  ament_lint_cmake()
  ament_uncrustify(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mocap4r2_vicon_driver test/test_mocap4r2_vicon_driver.cpp)
  target_link_libraries(test_mocap4r2_vicon_driver ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
