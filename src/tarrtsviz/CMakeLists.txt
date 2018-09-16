set(LIB ${PROJECT_NAME}_tarrts_viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            MLRRTstarPathPlanningInfo.cpp
            MLRRTstarViz.cpp
           )

target_link_libraries(${LIB}
                      ${PROJECT_NAME}_tarrts
                      ${QT_LIBRARIES}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                     )

