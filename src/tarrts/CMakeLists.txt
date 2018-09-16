
set(LIB ${PROJECT_NAME}_tarrts)

add_library(${LIB} SHARED
            MLRRTstar.cpp
            ExpandingTree.cpp
            ExpandingTreeMgr.cpp
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
