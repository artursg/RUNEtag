cmake_minimum_required(VERSION 2.8.11)

project(RUNETagGenerator)

set(CMAKE_LIBRARY_PATH "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../cmake/Modules/")

# Setup default installation dir
#
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../../dist" CACHE PATH "Install prefix" FORCE )
ENDIF()

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DQT_NO_DEBUG)
endif()

FIND_PACKAGE( OpenCV REQUIRED PATHS OpenCV_DIR NO_DEFAULT_PATH)
IF(OpenCV_INCLUDE_DIRS)
	MESSAGE(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")
	MESSAGE(STATUS "OpenCV library: ${OpenCV_LIB_DIR}")
	MESSAGE(STATUS " use OpenCV_DIR path variable to override")
ELSE(OpenCV_INCLUDE_DIRS)
	MESSAGE(FATAL_ERROR " OpenCV not found. Please set OpenCV_DIR path variable")
ENDIF(OpenCV_INCLUDE_DIRS)

# Ensure that POCO is available
#if( POCO_FOUNDATION_INCLUDE_DIRS )
#	MESSAGE( STATUS " POCO_FOUNDATION_INCLUDE_DIRS: ${POCO_FOUNDATION_INCLUDE_DIRS}" )
#else()
#	MESSAGE( ERROR " You must set POCO_FOUNDATION_INCLUDE_DIRS  PATH variable" )
#endif()
#if( POCO_FOUNDATION_LIBRARY )
#	MESSAGE( STATUS " POCO_INCLUDE_LIBS: ${POCO_FOUNDATION_LIBRARY}" )
#else()
#	MESSAGE( ERROR " You must set POCO_FOUNDATION_LIBRARY  FILE variable" )
#endif()


#find_package(Qwt REQUIRED)

#
#   CMAKE_PREFIX_PATH  is an environment variable (which can be set also via "set" cmake
#   command) that specifies the initial search location for find_package command.
#
#
#
#set (CMAKE_PREFIX_PATH "C:\\Program Files (x86)\\Windows Kits\\8.0\\Lib\\win8\\um\\x64")
#MESSAGE(STATUS " remember to add window SDK path to CMAKE_PREFIX_PATH!")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)
# Find the QtGui library
find_package(Qt5Gui)
find_package(Qt5PrintSupport)
# Find the QtGui library
#find_package(Qt5OpenGL)
#find_package(OpenGL)
#find_package(Qt5Network)
#find_package(Qt5Concurrent)

# List here all the ui files
QT5_WRAP_UI(UIS_HDRS
			runetaggenerator.ui
)

QT5_ADD_RESOURCES (ResourceSources 
			       runetaggenerator.qrc)

INCLUDE_DIRECTORIES( 
					 "${PROJECT_SOURCE_DIR}/../../dist/include"
				     ${OpenCV_INCLUDE_DIRS} 
					 )
#                     ${LIBMULTISPEC_CONF_INCLUDE_DIR}
#                     ${LIBMULTISPEC_INCLUDE_DIR}
#                     ${Boost_INCLUDE_DIRS}
#					 ${QWT_INCLUDE_DIR}
#					 ${POCO_FOUNDATION_INCLUDE_DIRS}		
#					 )
					 
#message( STATUS "QWT_INCLUDE_DIR:  ${QWT_INCLUDE_DIR}")

#message( info, "libMultiSpec conf inc dir: ${LIBMULTISPEC_CONF_INCLUDE_DIR}")
#message( info, "libMultiSpec inc dir: ${LIBMULTISPEC_INCLUDE_DIR}")


add_executable(RUNETagGenerator WIN32 
						main.cpp
						MarkPoint.cpp
						MarkPoint.h
						MarkPointSet.cpp
						MarkPointSet.h
						runetaggenerator.cpp
						runetaggenerator.h
						TagCode.cpp
						TagCode.h						
						${UIS_HDRS}
						${ResourceSources}
					   )

							   

target_link_libraries(RUNETagGenerator Qt5::Widgets Qt5::Gui Qt5::PrintSupport
								 ${OpenCV_LIBS} 
								 "${PROJECT_SOURCE_DIR}/../../dist/lib/libRUNETag.lib"
								 "${PROJECT_SOURCE_DIR}/../../dist/lib/WinNTL.lib"
								 #${Boost_LIBRARIES} 
								 #${QWT_LIBRARY}
								 #${POCO_FOUNDATION_LIBRARY}
								 )
								 
install( TARGETS RUNETagGenerator DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
