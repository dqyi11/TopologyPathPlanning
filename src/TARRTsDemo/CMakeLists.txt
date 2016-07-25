set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tpp-tarrts-demo
               mlrrtstar_config.h
               mlrrtstar_config.cpp
               mlrrtstar_window.h
               mlrrtstar_window.cpp
               mlrrtstar_demo.cpp
              )

target_link_libraries(tpp-tarrts-demo
                      tpp-homotopy
                      tpp-tarrts
                      tpp-tarrts-viz
                      ${QT_LIBRARIES}
                     )

