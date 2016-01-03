set(LIB tbpp-tarrts-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HDR    mlrrtstar_path_planning_info.h
           mlrrtstar_viz.h
           )

add_library(${LIB} SHARED
            mlrrtstar_path_planning_info.cc
            mlrrtstar_viz.cc
           )

target_link_libraries(${LIB}
                      tbpp-tarrts
                      ${QT_LIBRARIES}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                     )

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(FILES ${HDR}
        DESTINATION include/) 

