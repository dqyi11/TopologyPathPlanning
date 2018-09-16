
set(LIB ${PROJECT_NAME}_harrts)

add_library(${LIB} SHARED
            BiRRTstar.cpp
            StringClassMgr.cpp
           )

include_directories(
                   )

target_link_libraries(${LIB}
                      ${PROJECT_NAME}_homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     )

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)
