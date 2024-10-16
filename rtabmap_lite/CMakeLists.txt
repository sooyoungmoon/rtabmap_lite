cmake_minimum_required(VERSION 3.5)
PROJECT( rtabmap_lite )

# Add the -fPIC flag for position independent code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(realsense2 REQUIRED)
find_package(RTABMap REQUIRED) # ok
find_package(VTK REQUIRED)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
include_directories(${realsense2_INCLUDE_DIR})
include_directories(${RTABMap_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})


link_directories(${realsense2_LIBRARY_DIR})


add_executable(rtabmap_lite_node src/main.cpp src/RTABMapLiteNode.cpp)
target_link_libraries(rtabmap_lite_node rtabmap::gui ${VTK_LIBRARIES})


ament_target_dependencies(rtabmap_lite_node rclcpp std_msgs)
set_target_properties(rtabmap_lite_node PROPERTIES OUTPUT_NAME "rtabmap_lite_node")

install(TARGETS 
  rtabmap_lite_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


