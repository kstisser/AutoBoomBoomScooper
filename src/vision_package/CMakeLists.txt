cmake_minimum_required(VERSION 3.8)
project(vision_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

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

include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  "/opt/ros/jazzy/include"
  ) 

add_executable(camera_manager_node 
  src/main.cpp
  src/camera_manager.cpp
  )

ament_target_dependencies(camera_manager_node 
rclcpp
sensor_msgs
cv_bridge
image_transport
)
message(STATUS "cv_bridge include dirs: ${cv_bridge_INCLUDE_DIRS}")

target_link_libraries(camera_manager_node
  realsense2::realsense2
)

install(TARGETS 
  camera_manager_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
