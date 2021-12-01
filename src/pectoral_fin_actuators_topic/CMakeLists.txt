cmake_minimum_required(VERSION 3.0.2)

project(pectoral_fin_actuators_topic)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pectoral_fin_actuators_topic
  CATKIN_DEPENDS roscpp
  #DEPENDS stat ioctl unistd i2c i2c-dev stdio fcntl sys log inttypes errno math
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(pectoral_fin_actuators_topic
	src/actuator.cpp
	src/PCA9685.cpp)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#	src/${PROJECT_NAME}/leftActuator.cpp
#	src/${PROJECT_NAME}/rightActuator.cpp
#	src/${PROJECT_NAME}/PCA9685.cpp
# )

install(TARGETS ${TARGET} CONFIGURATIONS Release RUNTIME DESTINATION "$ENV{HOME}/")
#target_link_libraries(${PROJECT_NAME} 
	#${catkin_LIBRARIES}
	#stat
	#ioctl
	#i2c-dev
#)
set(LIBPATH /usr/local /usr/local/include/)
#target_link_libraries(${PROJECT_NAME} 
#	${catkin_LIBRARIES}
#)
find_library(I2C_LIB i2c Paths${LIBPATH} REQUIRED)
if (I2C_LIB)
	message(STATUS "Found ${I2C_LIB}")
else()
	message(SEND_ERROR,"Failed to find i2c_lib")
endif()

find_library(STAT_LIB stat Paths${LIBPATH} REQUIRED)
if (STAT_LIB)
	message(STATUS "Found ${STAT_LIB}")
else()
	message(SEND_ERROR,"Failed to find stat_lib")
endif()

find_library(IOCTL_LIB ioctl Paths${LIBPATH} REQUIRED)
if (ioctl_LIB)
	message(STATUS "Found ${ioctl_LIB}")
else()
	message(SEND_ERROR,"Failed to find ioctl_lib")
endif()

find_library(I2CDEV_LIB stat Paths${LIBPATH} REQUIRED)
if (I2CDEV_LIB)
	message(STATUS "Found ${I2CDEV_LIB}")
else()
	message(SEND_ERROR,"Failed to find I2CDEV_lib")
endif()



install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

