set(LIB harrts-birrts-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HDR  path_planning_info.h
         birrtstarviz.h
         )

add_library(${LIB} SHARED
            path_planning_info.cc
            birrtstarviz.cc
           )


target_link_libraries(${LIB} 
                      harrts-birrts
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
