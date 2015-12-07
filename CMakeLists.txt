# The name of my project is "Kinect2 Grabber". CMakeLists files in this project can 
# refer to the root source directory of the project as ${PEOPLETRAINER_SOURCE_DIR} and 
# to the root binary directory of the project as ${PEOPLETRAINER_BINARY_DIR}. 
cmake_minimum_required (VERSION 2.8) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# The version number.
# must go before the project command
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
if(DEFINED CMAKE_BUILD_TYPE)
  set_property( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES} )
endif()
set (Kinect2 Grabber_VERSION_MAJOR 0)
set (Kinect2 Grabber_VERSION_MINOR 1)
set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

project ("Kinect2 Grabber")

find_package(OpenCV REQUIRED)

# Kinect2 SDK
SET(KinectSDK_ROOT_DIR "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/")
message(${KinectSDK_ROOT_DIR})
SET(KinectSDK_LIB_DIR "${KinectSDK_ROOT_DIR}lib/x64")
SET(KinectSDK_INCLUDE_DIR "${KinectSDK_ROOT_DIR}inc")
SET(KinectSDK_DLLS )
SET(KinectSDK_LIBRARIES 
	"${KinectSDK_LIB_DIR}/Kinect20.lib"
)
SET(KinectSDK_INCLUDES 
	"${KinectSDK_INCLUDE_DIR}/Kinect.h"
)

####################   Macro   #######################
MACRO(CHECK_FILES _FILES _DIR)
	SET(_MISSING_FILES)
	FOREACH(_FILE ${${_FILES}})
		IF(NOT EXISTS "${_FILE}")
			SET(KinectSDK_FOUND NO)
			get_filename_component(_FILE ${_FILE} NAME)
			SET(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
		ENDIF()
	ENDFOREACH()
	IF(_MISSING_FILES)
		MESSAGE(STATUS "In folder \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
		SET(KinectSDK_FOUND NO)
	ENDIF()
ENDMACRO(CHECK_FILES)

MACRO(CHECK_DIR _DIR)
	IF(NOT EXISTS "${${_DIR}}")
		MESSAGE(STATUS "Folder \"${${_DIR}}\" not found.")
		SET(KinectSDK_FOUND NO)
	ENDIF()
ENDMACRO(CHECK_DIR)

##################### Checking #######################
MESSAGE(STATUS "Searching KinectSDK.")
SET(KinectSDK_FOUND YES)

CHECK_DIR(KinectSDK_ROOT_DIR)
IF(KinectSDK_FOUND)
	CHECK_DIR(KinectSDK_LIB_DIR)
	CHECK_DIR(KinectSDK_INCLUDE_DIR)
	
	IF(KinectSDK_FOUND)
		#CHECK_FILES(KinectSDK_DLLS KinectSDK_ROOT_DIR)
		CHECK_FILES(KinectSDK_LIBRARIES KinectSDK_LIB_DIR)
		CHECK_FILES(KinectSDK_INCLUDES KinectSDK_INCLUDE_DIR)
	ENDIF()
ENDIF()

MESSAGE(STATUS "KinectSDK_FOUND - ${KinectSDK_FOUND}.")

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${KinectSDK_INCLUDE_DIR} )
include_directories( "include")

add_executable(Kinect2Grabber src/Kinect2Manager.cpp src/Main.cpp) 
target_link_libraries(Kinect2Grabber ${OpenCV_LIBS})
target_link_libraries(Kinect2Grabber ${KinectSDK_LIBRARIES})

install(TARGETS Kinect2Grabber DESTINATION bin)
