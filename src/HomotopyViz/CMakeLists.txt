
set(LIB homotopy-viz)


add_library(${LIB} SHARED
            homotopyviz.h
            homotopyviz.cc)

include_directories(
                   ${Qt5Widgets_INCLUDE_DIRS}
                   )

target_link_libraries(${LIB} 
                      homotopy
                      ${Qt5Widgets_LIBRARIES} 
                      ${OpenCV_LIBRARIES}
                     )

