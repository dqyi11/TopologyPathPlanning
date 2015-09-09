set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(homotopy-demo
               mainwindow.h
               mainwindow.cc
               homotopyVizDemo.cc) 

include_directories(
                   ${Qt5Widgets_INCLUDE_DIRS}
                   )

target_link_libraries(homotopy-demo 
                      homotopy-viz
                      ${Qt5Widgets_LIBRARIES}
                     ) 
