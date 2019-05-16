# Project
SET(PROJECT_NAME davis_sensor_save_events_ros)
PROJECT(${PROJECT_NAME} CXX C)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

# Custom CMake base module
SET(CMAKE_MODULE_PATH 
	${CMAKE_MODULE_PATH}
	${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/caer
	/usr/${CMAKE_INSTALL_DATAROOTDIR}/caer
	/usr/local/${CMAKE_INSTALL_DATAROOTDIR}/caer
	${CMAKE_INSTALL_PREFIX}/share/caer
	/usr/share/caer
	/usr/local/share/caer
)

# Basic setup for cAER
INCLUDE(caer-base)
CAER_SETUP(TRUE)

# Search for external libraries with pkg-config.
INCLUDE(FindPkgConfig)

# Basic device and threads support.
SET(CAER_INCDIRS "${CAER_LOCAL_INCDIRS}")
SET(CAER_LIBDIRS "${CAER_LOCAL_LIBDIRS}")

PKG_CHECK_MODULES(LIBCAER REQUIRED libcaer>=3.1.0)
SET(CAER_PKGCONFIG_REQUIRES_PRIVATE "libcaer >= 3.1.0")

SET(CAER_INCDIRS ${CAER_INCDIRS} ${LIBCAER_INCLUDE_DIRS})
SET(CAER_LIBDIRS ${CAER_LIBDIRS} ${LIBCAER_LIBRARY_DIRS})
SET(CAER_LIBS ${LIBCAER_LIBRARIES} ${CAER_C_THREAD_LIBS})

# Boost support for C++
FIND_PACKAGE(Boost 1.50 REQUIRED COMPONENTS system filesystem iostreams program_options)

SET(CAER_INCDIRS ${CAER_INCDIRS} ${Boost_INCLUDE_DIRS})
SET(CAER_LIBDIRS ${CAER_LIBDIRS} ${Boost_LIBRARY_DIRS})
SET(CAER_LIBS ${CAER_LIBS} ${Boost_LIBRARIES})

# IF ((NOT (Boost_MAJOR_VERSION EQUAL 1 AND Boost_MINOR_VERSION GREATER_EQUAL 61))
# 		OR (OS_UNIX OR OS_LINUX OR OS_MACOSX))
# 	# libdl needed if Boost too old to have Boost.DLL as a replacement,
# 	# or for Boost.DLL itself on some platforms like Linux and MacOS X.
# 	SET(CAER_LIBS ${CAER_LIBS} dl)
# ENDIF()

# Add plain cmake packages
FIND_PACKAGE(OpenCV REQUIRED)

SET(CAER_INCDIRS ${CAER_INCDIRS} ${OpenCV_INCLUDE_DIRS})
SET(CAER_LIBDIRS ${CAER_LIBDIRS} ${OpenCV_LIBRARY_DIRS})
SET(CAER_LIBS ${CAER_LIBS} ${OpenCV_LIBRARIES})

FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge roslib image_transport sensor_msgs std_msgs dvs_msgs nav_msgs geometry_msgs)
catkin_package()

SET(CAER_INCDIRS ${CAER_INCDIRS} ${catkin_INCLUDE_DIRS})
SET(CAER_LIBDIRS ${CAER_LIBDIRS} ${catkin_LIBRARY_DIRS})
SET(CAER_LIBS ${CAER_LIBS} ${catkin_LIBRARIES})

# Add local directory to include and library paths
SET(CAER_INCDIRS ${CAER_INCDIRS} ${CMAKE_SOURCE_DIR}/)
SET(CAER_LIBDIRS ${CAER_LIBDIRS} ${CMAKE_SOURCE_DIR}/)

# # make the executable
# cs_add_executable(davis_ros_driver
#   src/driver_node.cpp
#   src/driver.cpp
# )
# 
# # make the nodelet into a library
# cs_add_library(davis_ros_driver_nodelet
#   src/driver_nodelet.cpp
#   src/driver.cpp
# )

# Set include/library paths for successive compilation.
INCLUDE_DIRECTORIES(
	include
	${CAER_INCDIRS}
)

# Set link libraries
LIST(APPEND LINK_LIBS
	${CAER_LIBS}
)

# Set sourcefiles
LIST(APPEND SOURCEFILES
	src/main_script.cpp)

# Set build flags
set(CMAKE_CXX_FLAGS "-std=c++11 -pedantic -Wall -Wextra -O2 -lcaer")
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

################################################################################
ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LINK_LIBS})
