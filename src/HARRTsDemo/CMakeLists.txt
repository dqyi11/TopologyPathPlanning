set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(harrtstar-demo
               HARRTstarConfig.h
               HARRTstarConfig.cpp
               HARRTstarWindow.h
               HARRTstarWindow.cpp
               harrtstar_demo.cpp
               )

include_directories(
                   ${Qt5Widgets_INCLUDE_DIRS}
                   ${Qt5Core_INCLUDE_DIRS}
                   )

target_link_libraries(harrtstar-demo
                      harrtstar
                      harrtstar-viz
                      ${Qt5Widgets_LIBRARIES}
                      ${Qt5Core_LIBRARIES}
                     )
