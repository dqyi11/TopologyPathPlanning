set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(${PROJECT_NAME}_harrts_demo
               BiRRTstarConfig.cpp
               BiRRTstarWindow.cpp
               BiRRTstarDemo.cpp
               )

include_directories(
                   )

target_link_libraries(${PROJECT_NAME}_harrts_demo
                      ${PROJECT_NAME}_homotopy
                      ${PROJECT_NAME}_homotopy_viz
                      ${PROJECT_NAME}_harrts
                      ${PROJECT_NAME}_harrts_viz
                      ${QT_LIBRARIES}
                     )

