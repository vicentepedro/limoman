# Copyright: (C) 2015 Vis Lab - Instituto Superior Tecnico
# Authors: Lorenzo Jamone (lorejam)
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.4)

set(TARGETNAME gloveToYarp)
set(PROJECTNAME ${TARGETNAME})
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(libglove REQUIRED)

include_directories(${libglove_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${TARGETNAME} ${headers} ${sources})
target_link_libraries(${TARGETNAME} ${libglove_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${TARGETNAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
