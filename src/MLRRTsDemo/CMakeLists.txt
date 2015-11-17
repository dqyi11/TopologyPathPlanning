set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(harrts-mlrrts-demo
               mlrrtstar_config.h
               mlrrtstar_config.cpp
               mlrrtstar_window.h
               mlrrtstar_window.cpp
               mlrrtstar_demo.cpp
              )

target_link_libraries(harrts-mlrrts-demo
                      harrts-homotopy
                      harrts-mlrrts
                      harrts-mlrrts-viz
                      ${QT_LIBRARIES}
                     )

install(TARGETS harrts-mlrrts-demo DESTINATION bin)
