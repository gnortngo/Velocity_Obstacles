cmake_minimum_required(VERSION 3.8)
project(myrobot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add timer library
add_library(timer
  timer/timer.cpp
)
target_include_directories(timer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/timer>
  $<INSTALL_INTERFACE:timer>
)

# Declare the C++ executable
add_executable(diskrobot src/diskrobot.cpp)

# Specify libraries to link the executable target
ament_target_dependencies(diskrobot rclcpp visualization_msgs)

# Link timer library with diskrobot
target_link_libraries(diskrobot timer)

# Install targets
install(TARGETS
  diskrobot
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Add tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

