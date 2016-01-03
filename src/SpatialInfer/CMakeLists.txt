
set(LIB tbpp-spatial-infer)

set(HDR    spatial_relation_function.h
           inbetween_relation_function.h
           sideoof_relation_function.h
           avoid_relation_function.h
           spatial_relation_mgr.h
   )

add_library(${LIB} SHARED
            inbetween_relation_function.cc
            sideof_relation_function.cc
            avoid_relation_function.cc
            spatial_relation_mgr.cc
           )

target_link_libraries(${LIB}
                      tbpp-homotopy
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
