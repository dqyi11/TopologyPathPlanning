set(LIB harrtstar-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            path_planning_info.h
            path_planning_info.cc
            HARRTstarViz.h
            HARRTstarViz.cc
           )

include_directories(
                   )

target_link_libraries(${LIB} 
                      harrtstar
                      ${QT_LIBRARIES}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     )
install(TARGETS ${LIB} 
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

