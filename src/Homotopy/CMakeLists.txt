
set(LIB tpp-homotopy)

set(HDR  tpp/homotopy/obstacle.h
         tpp/homotopy/line_subsegment.h
         tpp/homotopy/region.h
         tpp/homotopy/string_grammar.h
         tpp/homotopy/homotopic_grammar.h
         tpp/homotopy/reference_frames.h
         tpp/homotopy/worldmap.h
         tpp/homotopy/cgal_util.h
         tpp/homotopy/world_datatype.h
         tpp/homotopy/img_load_util.h
         )

add_library(${LIB} SHARED
            obstacle.cc
            line_subsegment.cc
            region.cc
            string_grammar.cc
            homotopic_grammar.cc
            reference_frames.cc
            worldmap.cc
           )

target_link_libraries(${LIB}
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${LIBXML2_LIBRARIES}
                     )

add_custom_target(HomotopyHeaders SOURCES ${HDR})

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib
        RUNTIME 
        DESTINATION bin)

install(FILES ${HDR}
        DESTINATION  include/tpp/homotopy)
