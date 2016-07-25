set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tpp-spatial-infer-demo
               spatialinfer_window.h
               spatialinfer_window.cc
               spatialinfer_config.h
               spatialinfer_config.cc
               spatial_infer_demo.cc) 

include_directories(
                   )

target_link_libraries(tpp-spatial-infer-demo 
                      tpp-spatial-infer
                      tpp-spatial-infer-viz
                      ${QT_LIBRARIES}
                     ) 

