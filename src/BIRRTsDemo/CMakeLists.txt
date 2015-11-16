set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(harrts-birrts-demo
               BIRRTstarConfig.h
               BIRRTstarConfig.cpp
               BIRRTstarWindow.h
               BIRRTstarWindow.cpp
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
