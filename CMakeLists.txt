cmake_minimum_required(VERSION 3.5)
project(rrt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Catch2 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

## add_executable(tree_node src/TreeNode.cpp)
## add_executable(RRT src/RRT.cpp src/TreeNode.cpp)
add_executable(app src/Main.cpp src/MainWindow.ui src/MainWindow.cpp src/RRT.cpp src/TreeNode.cpp)

## ament_target_dependencies(tree_node Eigen3 Catch2)
## ament_target_dependencies(RRT Eigen3 Catch2)
ament_target_dependencies(app Eigen3 Catch2 Qt5)

## install(TARGETS tree_node DESTINATION lib/${PROJECT_NAME})
## install(TARGETS RRT DESTINATION lib/${PROJECT_NAME})
install(TARGETS app DESTINATION lib/${PROJECT_NAME})

## target_link_libraries(tree_node Catch2::Catch2)
## target_link_libraries(RRT Catch2::Catch2)
target_link_libraries(app PRIVATE Qt5::Core Qt5::Widgets Catch2::Catch2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
