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
                   )

target_link_libraries(harrtstar-demo
                      homotopy
                      homotopy-viz
                      harrtstar
                      harrtstar-viz
                      ${QT_LIBRARIES}
                     )
