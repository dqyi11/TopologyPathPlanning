
set(LIB ${PROJECT_NAME}_homotopy)

add_library(${LIB} SHARED
            Obstacle.cpp
            LineSubsegment.cpp
            Region.cpp
            StringGrammar.cpp
            HomotopicGrammar.cpp
            ReferenceFrames.cpp
            Worldmap.cpp
            ImgLoadUtil.cpp
           )

target_link_libraries(${LIB}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${LIBXML2_LIBRARIES}
                     )


install(TARGETS ${LIB}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

