set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(harrts-homotopy-demo
               mainwindow.h
               mainwindow.cc
               homotopyVizDemo.cc) 

include_directories(
                   )

target_link_libraries(harrts-homotopy-demo 
                      harrts-homotopy-viz
                      ${QT_LIBRARIES}
                     ) 

install(TARGETS harrts-homotopy-demo DESTINATION bin)
