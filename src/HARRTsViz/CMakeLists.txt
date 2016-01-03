set(LIB tbpp-harrts-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HDR  birrtstar_path_planning_info.h
         birrtstar_viz.h
         )

add_library(${LIB} SHARED
            birrtstar_path_planning_info.cc
            birrtstar_viz.cc
           )


target_link_libraries(${LIB} 
                      tbpp-harrts
                      ${QT_LIBRARIES}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     )
install(TARGETS ${LIB} 
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(FILES ${HDR}
        DESTINATION include/)
