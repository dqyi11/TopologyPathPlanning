
set(LIB tbpp-spatial-infer)

set(HDR    spatial_relation_function.h
           inbetween_relation_function.h
   )

add_library(${LIB} SHARED
            inbetween_relation_function.cc
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
