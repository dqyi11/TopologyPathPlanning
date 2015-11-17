set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(harrts-birrts-demo
               birrtstar_config.h
               birrtstar_config.cpp
               birrtstar_window.h
               birrtstar_window.cpp
               birrtstar_demo.cpp
               )

include_directories(
                   )

target_link_libraries(harrts-birrts-demo
                      harrts-homotopy
                      harrts-homotopy-viz
                      harrts-birrts
                      harrts-birrts-viz
                      harrts-mlrrts
                      ${QT_LIBRARIES}
                     )
install(TARGETS harrts-birrts-demo DESTINATION bin)
