
set(LIB harrtstar)

set(HDR HARRTstar.h
        string_class_mgr.h
        KDTree2D.h
        kdtree++/allocator.hpp
        kdtree++/function.hpp
        kdtree++/iterator.hpp
        kdtree++/kdtree.hpp
        kdtree++/node.hpp
        kdtree++/region.hpp
        )

add_library(${LIB} SHARED
            HARRTstar.cc
            string_class_mgr.cc
           )

target_link_libraries(${LIB}
                      homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 
install(FILES ${HDR} DESTINATION include/)

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)
