
set(LIB rrtstar)

add_library(${LIB} SHARED
            kdtree++/allocator.hpp
            kdtree++/function.hpp
            kdtree++/iterator.hpp
            kdtree++/kdtree.hpp
            kdtree++/node.hpp
            kdtree++/region.hpp
            BiRRTstar.h
            BiRRTstar.cc
           )

target_link_libraries(${LIB})
