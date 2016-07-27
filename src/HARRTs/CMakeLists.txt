
set(LIB tpp-harrts)

set(HDR tpp/harrt/birrtstar.h
        tpp/harrt/string_class_mgr.h
        tpp/harrt/kdtree2d.h
        )

set(KD_HDR tpp/harrt/kdtree++/allocator.hpp
           tpp/harrt/kdtree++/function.hpp
           tpp/harrt/kdtree++/iterator.hpp
           tpp/harrt/kdtree++/kdtree.hpp
           tpp/harrt/kdtree++/node.hpp
           tpp/harrt/kdtree++/region.hpp
        )

add_library(${LIB} SHARED
            birrtstar.cc
            string_class_mgr.cc
           )

include_directories(
                   )

target_link_libraries(${LIB}
                      tpp-homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 

install(FILES ${HDR} DESTINATION include/tpp/harrt/)
install(FILES ${KD_HDR} DESTINATION include/tpp/harrt/kdtree++/)

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)
