set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(homotopy-viz-demo
               mainwindow.h
               mainwindow.cc
               homotopyVizDemo.cc) 
target_link_libraries(homotopy-viz-demo homotopy-viz ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}) 
