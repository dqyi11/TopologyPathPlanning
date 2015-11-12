
set(LIB harrts-mlrrts)

add_library(${LIB} SHARED
            MLRRTstar.h
            MLRRTstar.cc
           )

target_link_libraries(${LIB}
                      harrts-homotopy
                      harrts-birrts
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${HARRT_LIBRARIES}
                     ) 
