
set(LIB harrts-mlrrts)

set(HDR    mlrrtstar.h
           expanding_tree.h
           kdtreeml2d.h
   )

add_library(${LIB} SHARED
            mlrrtstar.cc 
            expanding_tree.cc
           )

target_link_libraries(${LIB}
                      harrts-homotopy
                      harrts-birrts
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
