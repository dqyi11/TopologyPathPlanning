set(CMAKE_BUILD_WITH_INSTALL_PATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
 
# search for the libxml2 package, exit if not found
find_package(LibXml2)
if( NOT LIBXML2_FOUND )
  message( FATAL_ERROR "libxml2 not found\n" )
  return()
endif()
include_directories(${LIBXML2_INCLUDE_DIR})

find_package(Boost COMPONENTS graph REQUIRED)
if( NOT Boost_FOUND )
  message( FATAL_ERROR "Boost not found\n" )
  return()
endif()
include_directories(${Boost_INCLUDE_DIR})

# search for CGAL, exit if not found
find_package(CGAL QUIET COMPONENTS Core)
if( NOT CGAL_FOUND )
  message( FATAL_ERROR "CGAL not found\n" ) 
  return()
endif()
#message( "CGAL FOUND ")
#message( "CGAL_USE_FILE: " ${CGAL_USE_FILE} )
#message( "CGAL_INCLUDE_DIR: " ${CGAL_INCLUDE_DIR} )
include(${CGAL_USE_FILE})
include_directories(${CGAL_INCLUDE_DIR})

#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

# search for the QT package, exit if not found
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
if( NOT QT4_FOUND ) 
  message( FATAL_ERROR "Qt4 not found\n" )
  return()
endif()
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR})

# search for the Graphviz package, exit if not found
#find_package(Graphviz REQUIRED)
#if( NOT Graphviz_FOUND )
#  message( FATAL_ERROR "Graphviz not found\n" )
#  return()
#endif()
#include_directories(${Graphviz_INCLUDE_DIR}) 

# search for Open CV, exit if not found
find_package( OpenCV REQUIRED )
if( NOT OpenCV_FOUND ) 
  message( FATAL_ERROR "OpenCV not found\n")
  return()
endif()
#message( "OpenCV FOUND ")
#message( "OpenCV_USE_FILE: " ${OpenCV_USE_FILE} )
#message( "OpenCV_INCLUDE_DIR: " ${OpenCV_INCLUDE_DIRS} )
#message( "OpenCV_LIB:" ${OpenCV_LIBRARIES} )
include_directories(${OpenCV_INCLUDE_DIRS})


#add the include directories to the build
include_directories(${PROJECT_SOURCE_DIR}/src/Homotopy
                    ${PROJECT_SOURCE_DIR}/src/HomotopyViz 
                    ${PROJECT_SOURCE_DIR}/src/HomotopyDemo
                    ${PROJECT_SOURCE_DIR}/src/HARRTs
                    ${PROJECT_SOURCE_DIR}/src/HARRTsViz
                    ${PROJECT_SOURCE_DIR}/src/HARRTsDemo
                    ${PROJECT_SOURCE_DIR}/src/TARRTs
                    ${PROJECT_SOURCE_DIR}/src/TARRTsViz
                    ${PROJECT_SOURCE_DIR}/src/TARRTsDemo
                   )

add_definitions(-Wall -g -O3)

add_subdirectory(Homotopy)
add_subdirectory(HomotopyViz)
add_subdirectory(HomotopyDemo)
add_subdirectory(HARRTs)
add_subdirectory(HARRTsViz)
add_subdirectory(HARRTsDemo)
add_subdirectory(TARRTs)
add_subdirectory(TARRTsViz)
add_subdirectory(TARRTsDemo)

