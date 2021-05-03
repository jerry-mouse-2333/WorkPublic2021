cmake_minimum_required(VERSION 3.0)
set(PROJECT_NAME lane_cali)
project(${PROJECT_NAME})

add_compile_options(-std=c++11)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 QUIET)
#MESSAGE("pcl_include_dir:" ${PCL_LIBRARIES})
find_package(catkin REQUIRED COMPONENTS
  pcl_ros 
  roscpp
  geometry_msgs
  autoware_msgs
)
catkin_package(
INCLUDE_DIRS include
  CATKIN_DEPENDS  geometry_msgs  pcl pcl_conversation pcl_ros ros roscpp
 # DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}   
)

add_library(lane_cali_ src/lane_cali.cpp src/assit.cpp)
target_link_libraries(lane_cali_ ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} )


add_executable(${PROJECT_NAME} node/lane_cali_node.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} lane_cali_)

#install(TARGETS {PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


