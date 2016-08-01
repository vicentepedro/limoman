# Copyright: (C) 2015 Vis Lab - Instituto Superior Tecnico
# Authors: Lorenzo Jamone (lorejam)
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME handControl)
project(${PROJECTNAME})


# find YARP and ICUB
FIND_PACKAGE(YARP)
# FIND_PACKAGE(ICUB)

# add include directories
# INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} )

# add required linker flags
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}") 
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")             

ADD_EXECUTABLE(${PROJECTNAME} src/handControl.cpp src/handControlMain.cpp include/handControl.h)

# we now add the YARP and iCub libraries to our project.
# TARGET_LINK_LIBRARIES(dynamicControl ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION $ENV{ICUB_DIR}/bin)

