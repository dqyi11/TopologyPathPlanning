set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tbpp-homotopy-demo
               mainwindow.h
               mainwindow.cc
               homotopyVizDemo.cc) 

include_directories(
                   )

target_link_libraries(tbpp-homotopy-demo 
                      tbpp-homotopy-viz
                      ${QT_LIBRARIES}
                     ) 

install(TARGETS tbpp-homotopy-demo DESTINATION bin)
