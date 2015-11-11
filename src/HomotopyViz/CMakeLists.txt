
set(LIB harrts-homotopy-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            homotopyviz.h
            homotopyviz.cc)

include_directories(
                   )

target_link_libraries(${LIB} 
                      harrts-homotopy
                      ${QT_LIBRARIES} 
                      ${OpenCV_LIBRARIES}
                     )
install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

