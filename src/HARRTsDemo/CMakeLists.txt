set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(tbpp-harrts-demo
               birrtstar_config.h
               birrtstar_config.cpp
               birrtstar_window.h
               birrtstar_window.cpp
               birrtstar_demo.cpp
               )

include_directories(
                   )

target_link_libraries(tbpp-harrts-demo
                      tbpp-homotopy
                      tbpp-homotopy-viz
                      tbpp-harrts
                      tbpp-harrts-viz
                      #tbpp-tarrts
                      ${QT_LIBRARIES}
                     )
install(TARGETS tbpp-harrts-demo DESTINATION bin)
