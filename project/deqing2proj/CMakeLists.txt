cmake_minimum_required(VERSION 2.8.3)
project(radar_deqing2proj)

add_compile_options(-std=c++11)

find_package(radar_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  radar_msgs
  log_msgs
)
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS} -lm -lpthread")

catkin_package( CATKIN_DEPENDS message_runtime radar_msgs)


set(PROJECT_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_LIB_DIR})

# set(USE_SONDIT_LIBRARY off)
# if (USE_SONDIT_LIBRARY)
# include_directories(include/sondit_parser ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${radar_msgs_INCLUDE_DIRS})
# add_library(collect ${PROJECT_SOURCE_DIR}/src/mec_radar_parser.cpp ${PROJECT_SOURCE_DIR}/src/convert_parser.cpp ${PROJECT_SOURCE_DIR}/src/sondit_callback.cpp)
# target_link_libraries(collect	  ${PROJECT_SOURCE_DIR}/lib/libParser.so jsoncpp)
# add_executable(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/node/radar_deqing2proj_parser_node.cpp)
# else()
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${radar_msgs_INCLUDE_DIRS})
add_library(collect ${PROJECT_SOURCE_DIR}/src/mec_radar.cpp ${PROJECT_SOURCE_DIR}/src/convert.cpp  ${PROJECT_SOURCE_DIR}/src/radar_log.cpp)
target_link_libraries(collect	  jsoncpp)
add_executable(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/node/radar_deqing2proj_node.cpp)
# endif()


target_link_libraries(${PROJECT_NAME}_node 	  ${catkin_LIBRARIES}  collect )
add_dependencies(${PROJECT_NAME}_node  ${catkin_EXPORTED_TARGETS})

install(TARGETS   ${PROJECT_NAME}_node collect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES   ${PROJECT_SOURCE_DIR}/launch/${PROJECT_NAME}.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

