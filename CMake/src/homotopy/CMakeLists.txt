
set(LIB homotopy)

add_library(${LIB} SHARED
            obstacle.h
            obstacle.cc
            region.h
            region.cc
            worldmap.h
            worldmap.cc
            reference_frames.h
            reference_frames.cc
            world_datatype.h)

target_link_libraries(${LIB} ${CGAL_LIBRARIES} ${BOOST_LIBRARIES})

