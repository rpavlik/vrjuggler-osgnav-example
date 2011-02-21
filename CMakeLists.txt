# Minimal VR Juggler+OpenSourceGraph Build
# CMake cross-platform build system recipe
# 2009 Ryan Pavlik <rpavlik@iastate.edu>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC

cmake_minimum_required(VERSION 2.6)

# Set package properties
project(osgNav)

###
# CMake Modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(UseBackportedModules)
include(SearchProgramFilesForOpenSceneGraph)
include(CreateLaunchers)
include(GetDirectoryList)

###
# Dependencies

# VR Juggler
find_package(VRJuggler 3.0 REQUIRED)
include_directories(${VRJUGGLER_INCLUDE_DIRS})
list(APPEND RUNTIME_LIBRARY_DIRS ${VRJUGGLER_RUNTIME_LIBRARY_DIRS})
add_definitions(${VRJUGGLER_DEFINITIONS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VRJUGGLER_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VRJUGGLER_CXX_FLAGS}")

# OpenSceneGraph
find_package(OpenSceneGraph REQUIRED COMPONENTS osgUtil osgDB)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
get_directory_list(OPENSCENEGRAPH_RUNTIME_LIBRARY_DIRS ${OPENSCENEGRAPH_LIBRARIES})
list(APPEND RUNTIME_LIBRARY_DIRS ${OPENSCENEGRAPH_RUNTIME_LIBRARY_DIRS})
if(WIN32)
	list(APPEND RUNTIME_LIBRARY_DIRS ${OPENSCENEGRAPH_RUNTIME_LIBRARY_DIRS}/../bin)
endif()

###
# Build the project
###
set(SOURCES
	main.cpp
	nav.h
	OsgNav.cpp
	OsgNav.h)

add_executable(osgnav ${SOURCES})
target_link_libraries(osgnav
	${OPENSCENEGRAPH_LIBRARIES}
	${VRJUGGLER_LIBRARIES})
create_default_target_launcher(osgnav
	FORWARD_ARGS
	RUNTIME_LIBRARY_DIRS
	${RUNTIME_LIBRARY_DIRS}
	ENVIRONMENT
	${VRJUGGLER_ENVIRONMENT})
