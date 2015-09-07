
set(LIB harrtstar)

add_library(${LIB} SHARED
            kdtree++/allocator.hpp
            kdtree++/function.hpp
            kdtree++/iterator.hpp
            kdtree++/kdtree.hpp
            kdtree++/node.hpp
            kdtree++/region.hpp
            HARRTstar.h
            HARRTstar.cc
           )

target_link_libraries(${LIB}
                      homotopy
                      ${CGAL_LIBRARIES}
                      ${BOOST_LIBRARIES}
                     ) 
