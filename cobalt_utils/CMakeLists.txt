cmake_minimum_required(VERSION 3.5)
project(cobalt_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(odomTotransform_DEPENDS
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

add_executable(odomTotransform src/odomTotransform.cpp)
ament_target_dependencies(odomTotransform ${odomTotransform_DEPENDS})
target_include_directories(odomTotransform PUBLIC include)

install(TARGETS odomTotransform
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${odomTotransform_DEPENDS})
ament_package()