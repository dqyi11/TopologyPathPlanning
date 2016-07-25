set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tpp-homotopy-demo
               mainwindow.h
               mainwindow.cc
               homotopyVizDemo.cc) 

include_directories(
                   )

target_link_libraries(tpp-homotopy-demo 
                      tpp-homotopy-viz
                      ${QT_LIBRARIES}
                     )
