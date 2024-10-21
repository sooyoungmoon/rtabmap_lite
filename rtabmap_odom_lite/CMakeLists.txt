cmake_minimum_required(VERSION 3.5)
PROJECT( rtabmap_odom_lite )

# Add the -fPIC flag for position independent code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(realsense2 REQUIRED)
find_package(RTABMap REQUIRED) # ok
find_package(VTK REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
message(STATUS "TEST MESSAGE")



# message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${realsense2_INCLUDE_DIR})
include_directories(${RTABMap_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})
include_directories(${image_transport_INCLUDE_DIRS})


link_directories(${realsense2_LIBRARY_DIR})


add_executable(rtabmap_odom_lite_node src/main.cpp src/RTABMapOdomLiteNode.cpp)
target_link_libraries(rtabmap_odom_lite_node rtabmap::gui ${VTK_LIBRARIES})


ament_target_dependencies(
  rtabmap_odom_lite_node 
  rclcpp 
  std_msgs
  cv_bridge 
  image_transport
  sensor_msgs)

set_target_properties(rtabmap_odom_lite_node PROPERTIES OUTPUT_NAME "rtabmap_odom_lite_node")

install(TARGETS 
  rtabmap_odom_lite_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


