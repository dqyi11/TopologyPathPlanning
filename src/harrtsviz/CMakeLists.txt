set(LIB ${PROJECT_NAME}_harrts_viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


add_library(${LIB} SHARED
            BiRRTstarPathPlanningInfo.cpp
            BiRRTstarViz.cpp
            ${PROJECT_SOURCE_DIR}/include/topologyPathPlanning/harrtsviz/BiRRTstarViz.hpp
           )


target_link_libraries(${LIB} 
                      ${PROJECT_NAME}_harrts
                      ${QT_LIBRARIES}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     )

install(TARGETS ${LIB} 
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)
