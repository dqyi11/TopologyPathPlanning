
set(LIB homotopy)

set(HDR obstacle.h
         line_subsegment.h
         region.h
         string_grammar.h
         homotopic_grammar.h
         reference_frames.h
         worldmap.h)

add_library(${LIB} SHARED
            obstacle.cc
            line_subsegment.cc
            region.cc
            string_grammar.cc
            homotopic_grammar.cc
            reference_frames.cc
            worldmap.cc
            cgal_util.h
            world_datatype.h)

target_link_libraries(${LIB}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${LIBXML2_LIBRARIES}
                     )

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(FILES ${HDR}
        DESTINATION include/)
