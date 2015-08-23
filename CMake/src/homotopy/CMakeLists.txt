
set(LIB homotopy)

add_library(${LIB} SHARED
            obstacle.h
            obstacle.cc
            line_subsegment.h
            line_subsegment.cc
            region.h
            region.cc
            string_grammar.h
            string_grammar.cc
            homotopic_grammar.h
            homotopic_grammar.cc
            reference_frames.h
            reference_frames.cc
            worldmap.h
            worldmap.cc
            world_datatype.h)

target_link_libraries(${LIB} ${CGAL_LIBRARIES} ${BOOST_LIBRARIES} ${LIBXML2_LIBRARIES})

