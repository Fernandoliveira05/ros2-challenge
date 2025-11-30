cmake_minimum_required(VERSION 3.8)
project(cg_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(maze_solver src/solver.cpp)
add_executable(mapear src/mapear.cpp)
ament_target_dependencies(mapear rclcpp cg_interfaces)

ament_target_dependencies(maze_solver
  rclcpp
  cg_interfaces
)

install(TARGETS
  maze_solver
  mapear
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
