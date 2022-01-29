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
find_package(image_transport REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-msgs8	REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

set(odomTotransform_DEPENDS
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

set(robustImageBridge_DEPENDS
  rclcpp
  image_transport
  ros_ign_bridge
  sensor_msgs
)

set(pointCloudTransport_DEPENDS
  rclcpp
  image_transport
  ros_ign_bridge
  sensor_msgs
)

set(DEPENDS
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  image_transport
  ros_ign_bridge
  sensor_msgs
)

add_executable(odomTotransform src/odomTotransform.cpp)
ament_target_dependencies(odomTotransform ${odomTotransform_DEPENDS})
target_include_directories(odomTotransform PUBLIC include)

add_executable(robustImageBridge src/robustImageBridge.cpp)
ament_target_dependencies(robustImageBridge ${robustImageBridge_DEPENDS})
target_link_libraries(robustImageBridge
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)
target_include_directories(robustImageBridge PUBLIC include)

add_executable(pointCloudTransport src/pointCloudTransport.cpp)
ament_target_dependencies(pointCloudTransport ${pointCloudTransport_DEPENDS})
target_link_libraries(pointCloudTransport
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)
target_include_directories(pointCloudTransport PUBLIC include)

install(TARGETS odomTotransform robustImageBridge pointCloudTransport
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
ament_export_dependencies(${DEPENDS})
ament_package()