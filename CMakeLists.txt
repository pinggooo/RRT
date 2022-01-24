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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Catch2 REQUIRED)

## add_executable(tree_node src/TreeNode.cpp)
## add_executable(RRT src/RRT.cpp src/TreeNode.cpp)
add_executable(rrt_test src/Main.cpp src/RRT.cpp src/TreeNode.cpp src/Application.cpp)

## ament_target_dependencies(tree_node Eigen3 Catch2)
## ament_target_dependencies(RRT Eigen3 Catch2)
ament_target_dependencies(rrt_test rclcpp Eigen3 Catch2)

target_include_directories(rrt_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

## install(TARGETS tree_node DESTINATION lib/${PROJECT_NAME})
## install(TARGETS RRT DESTINATION lib/${PROJECT_NAME})
install(TARGETS rrt_test DESTINATION lib/${PROJECT_NAME})

## target_link_libraries(tree_node Catch2::Catch2)
## target_link_libraries(RRT Catch2::Catch2)
target_link_libraries(rrt_test Catch2::Catch2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
