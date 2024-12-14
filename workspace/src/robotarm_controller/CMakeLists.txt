cmake_minimum_required(VERSION 3.8)
project(robotarm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL libserial)

add_library(${PROJECT_NAME} SHARED
    src/robotarm_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    "${SERIAL_INCLUDE_DIRS}"
)

target_link_libraries(${PROJECT_NAME}
    "${SERIAL_LDFLAGS}"
)

ament_target_dependencies(${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface robotarm_controller.xml)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
    )

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
)

install(
    DIRECTORY include
    DESTINATION include
)

ament_export_include_directories(
    include
)

ament_export_libraries(
    ${PROJECT_NAME}
)

ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
