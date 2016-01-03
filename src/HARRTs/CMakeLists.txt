
set(LIB tbpp-harrts)

set(HDR birrtstar.h
        string_class_mgr.h
        KDTree2D.h
        )

set(KD_HDR kdtree++/allocator.hpp
           kdtree++/function.hpp
           kdtree++/iterator.hpp
           kdtree++/kdtree.hpp
           kdtree++/node.hpp
           kdtree++/region.hpp
        )

add_library(${LIB} SHARED
            birrtstar.cc
            string_class_mgr.cc
           )

include_directories(
                   )

target_link_libraries(${LIB}
                      tbpp-homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 

install(FILES ${HDR} DESTINATION include/)
install(FILES ${KD_HDR} DESTINATION include/kdtree++/)

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)
