cmake_minimum_required(VERSION 2.8.3)
project(sodahead_ctl)

find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp sensor_msgs trajectory_msgs roslib)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE(yaml-cpp REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_service_files(FILES
  CalibrateServo.srv
)

generate_messages()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES sodahead_driver sodahead
   CATKIN_DEPENDS message_runtime roscpp roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(sodahead src/sodahead.cpp)
target_link_libraries(sodahead ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(sodahead_driver src/sodahead_driver.cpp)
add_dependencies(sodahead_driver ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(sodahead_driver sodahead ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(sodahead_node src/sodahead_node.cpp)
target_link_libraries(sodahead_node sodahead_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(test_sodahead src/test_sodahead.cpp)
#target_link_libraries(test_sodahead sodahead_driver ${catkin_LIBRARIES})

install(TARGETS sodahead sodahead_driver sodahead_node #test_sodahead
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

