cmake_minimum_required(VERSION 3.10)
project(mocap4r2_vicon_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)

# Vicon DataStream SDK 1.13 (MIT, unmodified sources), static and internal.
set(VICON_SDK ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/ViconDataStreamSDK)
file(GLOB_RECURSE VICON_SDK_SOURCES ${VICON_SDK}/*.cpp)
add_library(vicon_datastream STATIC ${VICON_SDK_SOURCES})
target_include_directories(vicon_datastream SYSTEM PUBLIC
  ${VICON_SDK}
  ${VICON_SDK}/DataStream
  ${VICON_SDK}/DataStream/ViconDataStreamSDK_CPP
  ${ASIO_INCLUDE_DIR}
)
target_compile_definitions(vicon_datastream PRIVATE
  NDEBUG TCM_OFF_SITE TCM_LINUX TCM_UNIX RELEASE_SOFTWARE ASIO_STANDALONE=1
  PROJECT_SOURCE_PATH=\".\"
)
target_compile_options(vicon_datastream PRIVATE -w)
target_link_libraries(vicon_datastream PUBLIC Threads::Threads)

add_library(${PROJECT_NAME} SHARED
  src/sdk_vicon_source.cpp
  src/vicon_conversion.cpp
  src/vicon_driver.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mocap4r2_control::mocap4r2_control
  ${mocap4r2_interfaces_TARGETS}
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE vicon_datastream rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_vicon_driver::ViconDriver")

add_executable(vicon_driver src/vicon_driver_main.cpp)
target_link_libraries(vicon_driver ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES ThirdParty/ViconDataStreamSDK/LICENSE
  DESTINATION share/${PROJECT_NAME} RENAME LICENSE.ViconDataStreamSDK)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS vicon_driver RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Linters on our code only: the Vicon SDK is third-party code.
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  set(lint_paths include src test)
  ament_copyright(${lint_paths})
  ament_cppcheck(${lint_paths} LANGUAGE c++)
  ament_cpplint(${lint_paths})
  ament_lint_cmake()
  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
    # uncrustify 0.72 of Humble formats differently: style is checked on newer distros
    ament_uncrustify(${lint_paths} LANGUAGE c++)
  endif()
  ament_xmllint()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vicon_conversion test/test_vicon_conversion.cpp)
  target_link_libraries(test_vicon_conversion ${PROJECT_NAME})
  ament_add_gtest(test_vicon_driver test/test_vicon_driver.cpp TIMEOUT 120)
  target_link_libraries(test_vicon_driver ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mocap4r2_control mocap4r2_interfaces rclcpp)
ament_package()
