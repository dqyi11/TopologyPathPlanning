set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tbpp-tarrts-demo
               mlrrtstar_config.h
               mlrrtstar_config.cpp
               mlrrtstar_window.h
               mlrrtstar_window.cpp
               mlrrtstar_demo.cpp
              )

target_link_libraries(tbpp-tarrts-demo
                      tbpp-homotopy
                      tbpp-tarrts
                      tbpp-tarrts-viz
                      ${QT_LIBRARIES}
                     )

install(TARGETS tbpp-tarrts-demo DESTINATION bin)
