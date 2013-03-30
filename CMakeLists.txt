cmake_minimum_required(VERSION 2.8.3)
project(roboteq_nxtgen_controller)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs diagnostic_updater dynamic_reconfigure roscpp sensor_msgs std_srvs trajectory_msgs)

generate_dynamic_reconfigure_options(cfg/RoboteqNxtGen.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RoboteqDevice nxtgen_driver
  CATKIN_DEPENDS dynamic_reconfigure roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# RoboteqDevice standalone library
add_library(RoboteqDevice include/api_linux/Linux/RoboteqDevice.cpp)
get_target_property(RoboteqDevice_CFLAGS RoboteqDevice COMPILE_FLAGS)
set_target_properties(RoboteqDevice PROPERTIES COMPILE_FLAGS "${CFLAGS} -include unistd.h")
target_link_libraries(RoboteqDevice ${catkin_LIBRARIES})

# ROS library
add_library(nxtgen_driver src/nxtgen_driver.cpp)
target_link_libraries(nxtgen_driver RoboteqDevice ${catkin_LIBRARIES})
add_dependencies(nxtgen_driver ${PROJECT_NAME}_gencfg)

add_executable(nxtgen_driver_node src/nxtgen_driver_node.cpp)
target_link_libraries(nxtgen_driver_node nxtgen_driver ${catkin_LIBRARIES})
add_dependencies(nxtgen_driver_node ${PROJECT_NAME}_gencfg)

install(TARGETS RoboteqDevice nxtgen_driver
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS nxtgen_driver_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
