
set(LIB tpp-spatial-infer-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            spatialinfer_viz.h
            spatialinfer_viz.cc)

include_directories(
                   )

target_link_libraries(${LIB} 
                      tpp-homotopy
                      tpp-spatial-infer
                      ${QT_LIBRARIES} 
                      ${OpenCV_LIBRARIES}
                     )


