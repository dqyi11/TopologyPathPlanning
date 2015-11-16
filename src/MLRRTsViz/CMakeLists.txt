set(LIB harrts-mlrrts-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HDR    mlrrtstarviz.h
           )

add_library(${LIB} SHARED
            mlrrtstarviz.cc
           )

target_link_libraries(${LIB}
                      harrts-mlrrts
                      ${QT_LIBRARIES}
                     )

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(FILES ${HDR}
        DESTINATION include/) 

