set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tpp-harrts-demo
               birrtstar_config.h
               birrtstar_config.cpp
               birrtstar_window.h
               birrtstar_window.cpp
               birrtstar_demo.cpp
               )

include_directories(
                   )

target_link_libraries(tpp-harrts-demo
                      tpp-homotopy
                      tpp-homotopy-viz
                      tpp-harrts
                      tpp-harrts-viz
                      ${QT_LIBRARIES}
                     )

