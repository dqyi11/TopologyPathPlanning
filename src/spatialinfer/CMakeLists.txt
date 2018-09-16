
set(LIB ${PROJECT_NAME}_spatial_infer)

add_library(${LIB} SHARED
            InbetweenRelationFunction.cpp
            SideofRelationFunction.cpp
            AvoidRelationFunction.cpp
            SpatialRelationMgr.cpp
            Rule.cpp
            RuleSet.cpp
           )

target_link_libraries(${LIB}
                      ${PROJECT_NAME}_homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 



