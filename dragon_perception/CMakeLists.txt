cmake_minimum_required(VERSION 3.5)
project(dragon_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(occupancy_grid_map_node
  src/occupancy_grid_map_node.cpp
)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(occupancy_grid_map_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

install(FILES DESTINATION share/${PROJECT_NAME})
