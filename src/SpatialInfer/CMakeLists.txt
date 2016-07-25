
set(LIB tpp-spatial-infer)

set(HDR    tpp/spatial_infer/spatial_relation_function.h
           tpp/spatial_infer/inbetween_relation_function.h
           tpp/spatial_infer/sideoof_relation_function.h
           tpp/spatial_infer/avoid_relation_function.h
           tpp/spatial_infer/spatial_relation_mgr.h
           tpp/spatial_infer/rule.h
           tpp/spatial_infer/rule_set.h
   )

add_library(${LIB} SHARED
            inbetween_relation_function.cc
            sideof_relation_function.cc
            avoid_relation_function.cc
            spatial_relation_mgr.cc
            rule.cc
            rule_set.cc
           )

target_link_libraries(${LIB}
                      tpp-homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 



