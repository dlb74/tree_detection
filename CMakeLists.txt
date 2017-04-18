cmake_minimum_required(VERSION 2.8.3)
project(tree_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
  image_transport
)

find_package( PCL 1.7 REQUIRED )
find_package( OpenCV REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib


)




# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
)

include(${CGAL_USE_FILE})



add_executable(tree_detect src/main.cpp src/pcl_visualizer.cpp)
target_link_libraries(tree_detect 
	${catkin_LIBRARIES}
 	${CGAL_LIBRARIES} 
	${OpenCV_LIBRARIES} 
	${PCL_LIBRARIES} 
	${CGAL_LIBRARY} 
	${CGAL_Core_LIBRARY}
	${VTK_LIBRARIES})
	
add_executable(tree_detect_realtime src/realtimeDetect.cpp)
target_link_libraries(tree_detect_realtime 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES} )



#add_executable(cameraInfoOut src/cameraInfoOut.cpp src/ImagePublisher.cpp)
#target_link_libraries(cameraInfoOut ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
