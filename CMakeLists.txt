cmake_minimum_required(VERSION 2.8.3)
project(usv_mavros_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(Qt5Location REQUIRED)
find_package(Qt5Positioning REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(catkin REQUIRED COMPONENTS
mavros 
mavros_msgs 
mavlink
roscpp
std_msgs
geometry_msgs
message_generation
)

foreach(plugin ${Qt5Location_PLUGINS})
	get_target_property(_loc ${plugin}  LOCATION)
	message("Plugin ${$plugin} is at location ${_loc}")
endforeach()	
foreach(plugin ${Qt5Nwtwork_PLUGINS})
	get_target_property(_loc ${plugin}  LOCATION)
	message("Plugin ${$plugin} is at location ${_loc}")
endforeach()	

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake tfmavsasdaso run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS 
   include
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Qt5Core_DIR}
 ${Qt5Qml_DIR}
 ${Qt5Quick_DIR}
 ${Qt5Location_DIR}
 ${Qt5Positioning_DIR}
 ${Qt5Network_DIR}
)

set(MOC_HDRS include/mapmavrospipe.h)
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})
set(RCC_FILES mapviewer.qrc)
qt5_add_resources(RCC_SOURCES ${RCC_FILES})

### Declare the C++ Qt5 GUI executable
add_executable(${PROJECT_NAME}_node src/main.cpp src/mapmavrospipe.cpp src/waypointcontrol.cpp ${MOC_HDRS} ${RCC_SOURCES})
qt5_use_modules(${PROJECT_NAME}_node Core Qml Quick Location Positioning)

### Declare the C++ Qt5 NO GUI executable
add_executable(${PROJECT_NAME}_nogui_node src/sendwaypoints_nogui.cpp src/waypointcontrol.cpp)
add_executable(${PROJECT_NAME}_nogui_servo src/setActuators.cpp       src/waypointcontrol.cpp)
add_executable(${PROJECT_NAME}_nogui_torque src/usvSpeedTest.cpp      src/waypointcontrol.cpp)
add_executable(${PROJECT_NAME}_nogui_cmdvel src/usvCommandVel.cpp     src/waypointcontrol.cpp)

### Declare the waw ROS C++ library
add_library(${PROJECT_NAME} src/waypointcontrol.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_nogui_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_nogui_servo ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_nogui_torque ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_nogui_cmdvel ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
