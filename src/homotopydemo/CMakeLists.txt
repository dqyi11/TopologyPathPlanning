set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(${PROJECT_NAME}_homotopy_demo
               MainWindow.cpp
               HomotopyVizDemo.cpp)

include_directories(
                   )

target_link_libraries(${PROJECT_NAME}_homotopy_demo
                      ${PROJECT_NAME}_homotopy_viz
                      ${QT_LIBRARIES}
                     )

install(TARGETS ${BINS} DESTINATION bin)
