
set(LIB tpp-tarrts)

set(HDR    tpp/tarrt/mlrrtstar.h
           tpp/tarrt/expanding_tree.h
           tpp/tarrt/expanding_tree_mgr.h
           tpp/tarrt/kdtreeml2d.h
   )

add_library(${LIB} SHARED
            mlrrtstar.cc 
            expanding_tree.cc
            expanding_tree_mgr.cc
           )

target_link_libraries(${LIB}
                      tpp-homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 

add_custom_target(TARRTsHeaders SOURCES ${HDR})


install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(FILES ${HDR}
        DESTINATION include/tpp/tarrt/)
