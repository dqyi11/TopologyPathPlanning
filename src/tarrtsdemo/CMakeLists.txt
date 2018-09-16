set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(${PROJECT_NAME}_tarrts_demo
               MLRRTstarConfig.cpp
               MLRRTstarWindow.cpp
               MLRRTstarDemo.cpp
              )

target_link_libraries(${PROJECT_NAME}_tarrts_demo
                      ${PROJECT_NAME}_homotopy
                      ${PROJECT_NAME}_tarrts
                      ${PROJECT_NAME}_tarrts_viz
                      ${QT_LIBRARIES}
                     )

