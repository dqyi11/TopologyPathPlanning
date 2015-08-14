set(CMAKE_BUILD_WITH_INSTALL_PATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x)

find_package(Boost COMPONENTS)
include_directories(${Boost_INCLUDE_DIR})

# search for CGAL, exit if not found
find_package(CGAL COMPONENTS Core)
if( NOT CGAL_FOUND )
  message( FATAL_ERROR "CGAK not found\n" ) 
  return()
endif()
#message( "CGAL FOUND ")
#message( "CGAL_USE_FILE: " ${CGAL_USE_FILE} )
#message( "CGAL_INCLUDE_DIR: " ${CGAL_INCLUDE_DIR} )
include(${CGAL_USE_FILE})
include_directories(${CGAL_INCLUDE_DIR})

# search for the QT package, exit if not found
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
if( NOT QT4_FOUND )
  message( FATAL_ERROR "Qt4 not found\n")
  return()
endif()
#message( "QT4 FOUND ")
#message( "QT_USE_FILE: " ${QT_USE_FILE} )
#message( "QT_INCLUDE_DIR: " ${QT_INCLUDE_DIR} )
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR})

# search for Open CV, exit if not found
find_package( OpenCV REQUIRED )
if( NOT OpenCV_FOUND ) 
  message( FATA_ERROR "OpenCV not found\n")
  return()
endif()
#message( "OpenCV FOUND ")
#message( "OpenCV_USE_FILE: " ${OpenCV_USE_FILE} )
#message( "OpenCV_INCLUDE_DIR: " ${OpenCV_INCLUDE_DIRS} )
#message( "OpenCV_LIB:" ${OpenCV_LIBRARIES} )
include_directories(${OpenCV_INCLUDE_DIRS})


#add the include directories to the build
include_directories(${PROJECT_SOURCE_DIR}/src/homotopy
                    ${PROJECT_SOURCE_DIR}/src/homotopyViz 
                    ${PROJECT_SOURCE_DIR}/src/homotopyVizDemo
                    ${PROJECT_SOURCE_DIR}/src/HA-RRTs
                    ${PROJECT_SOURCE_DIR}/src/HA-RRTsVizDemo
                    ${PROJECT_SOURCE_DIR}/src/HA-RRTsDemo)


add_definitions(-Wall -g -O3)

add_subdirectory(homotopy)
add_subdirectory(homotopyViz)
add_subdirectory(homotopyVizDemo)
add_subdirectory(HA-RRTs)
add_subdirectory(HA-RRTsViz)
add_subdirectory(HA-RRTsDemo)
