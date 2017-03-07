CMAKE_MINIMUM_REQUIRED( VERSION 2.8.0 )
MARK_AS_ADVANCED( CMAKE_BACKWARDS_COMPATIBILITY )
SET( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )


PROJECT( libRUNETag )


OPTION( BUILD_SHARED_LIBS "Set to off to build static libraries" ON )
OPTION( BUILD_RUNETAGDETECT "Create RUNEtagdetect executable" ON )
OPTION( BUILD_CODEGEN "Create codegen executable" ON )


IF( NOT CMAKE_BUILD_TYPE )
	SET( CMAKE_BUILD_TYPE "Release" )
ENDIF()


IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../dist" CACHE PATH "Install prefix" FORCE )
ENDIF()


#
#	Dependencies
#
#FIND_PACKAGE( OpenCV REQUIRED PATHS OpenCV_DIR NO_DEFAULT_PATH)
#IF(OpenCV_INCLUDE_DIRS)
#	MESSAGE(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
#	MESSAGE(STATUS "OpenCV library: ${OpenCV_LIB_DIR}")
#	MESSAGE(STATUS " use OpenCV_DIR path variable to override")
#ELSE(OpenCV_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "OpenCV not found. Please set OpenCV_INCLUDE_DIRS and OpenCV_LIB_DIR path variables")
#ENDIF(OpenCV_INCLUDE_DIRS)
FIND_PACKAGE (OpenCV REQUIRED)

ADD_SUBDIRECTORY( WinNTL )


INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include"
					  ${OpenCV_INCLUDE_DIRS} 
					  "WinNTL/include"
					   )
					  
INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/include" DESTINATION . )
INSTALL( DIRECTORY "${PROJECT_SOURCE_DIR}/WinNTL/include" DESTINATION . )

SET( RUNETAG_SRCS 
		auxmath.cpp
		auxrenderer.cpp
		ellipsefitter.cpp
		ellipsepoint.cpp
		fpscounter.cpp
		hirestimer.cpp
		markerdetector.cpp
		markerpose.cpp
		slotfitter.cpp
		ellipserefine.cpp
		ellipsedetector.cpp
		digitalmarkermodel.cpp
		slot.cpp
		coding.cpp
		)
	 
SET( RUNETAG_INCL 
	  "include/auxmath.hpp"
	  "include/auxrenderer.hpp"
	  "include/digitalmarkermodel.hpp"
	  "include/ellipsefitter.hpp"
	  "include/ellipsepoint.hpp"
	  "include/fpscounter.hpp"
	  "include/hirestimer.hpp"
	  "include/imgtextstream.hpp"
	  "include/markerdetected.hpp"
	  "include/markerdetector.hpp"
	  "include/markerpose.hpp"
	  "include/slot.hpp"
	  "include/slotfitter.hpp"
	  "include/ellipserefine.hpp"
	  "include/ellipsedetector.hpp"
	  "include/runetag.hpp"
	  "include/coding.h")
	  

ADD_LIBRARY( ${PROJECT_NAME} STATIC ${RUNETAG_SRCS} ${RUNETAG_INCL} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} WinNTL ${OpenCV_LIBS})
SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION} CLEAN_DIRECT_OUTPUT 1 OUTPUT_NAME ${PROJECT_NAME} )


INSTALL( TARGETS ${PROJECT_NAME} DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )


IF( BUILD_RUNETAGDETECT )
ADD_SUBDIRECTORY( RUNEtagdetect )
ENDIF()





IF( BUILD_CODEGEN )
ADD_SUBDIRECTORY( codegen )
ENDIF()


IF( BUILD_VIRTUALCAMERA )
ADD_SUBDIRECTORY( virtualcamera )
ENDIF()


MESSAGE( STATUS )
MESSAGE( STATUS "Build Settings:" )
MESSAGE( STATUS "------------------------------------------------" )
MESSAGE( STATUS "BUILD_SHARED_LIBS\t\t= ${BUILD_SHARED_LIBS}" )
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX\t\t= ${CMAKE_INSTALL_PREFIX}" )
MESSAGE( STATUS "CMAKE_BUILD_TYPE\t\t= ${CMAKE_BUILD_TYPE}  (Options: Release, Debug, MinSizeRel, RelWithDebInfo)" )
MESSAGE( STATUS )
MESSAGE( STATUS "You can change this values with cmake -D<Variable>=<Value>" )
MESSAGE( STATUS "------------------------------------------------" )


