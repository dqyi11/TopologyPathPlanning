
set(LIB tbpp-tarrts)

set(HDR    mlrrtstar.h
           expanding_tree.h
           expanding_tree_mgr.h
           kdtreeml2d.h
   )

add_library(${LIB} SHARED
            mlrrtstar.cc 
            expanding_tree.cc
            expanding_tree_mgr.cc
           )

target_link_libraries(${LIB}
                      tbpp-homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${HARRT_LIBRARIES}
                     ) 


install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(FILES ${HDR}
        DESTINATION include/)
