set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tbpp-spatial-infer-demo
               spatialinfer_window.h
               spatialinfer_window.cc
               spatial_infer_demo.cc) 

include_directories(
                   )

target_link_libraries(tbpp-spatial-infer-demo 
                      tbpp-spatial-infer
                      tbpp-spatial-infer-viz
                      ${QT_LIBRARIES}
                     ) 

install(TARGETS tbpp-spatial-infer-demo DESTINATION bin)
