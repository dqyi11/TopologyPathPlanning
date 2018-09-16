
set(LIB ${PROJECT_NAME}_homotopy_viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            HomotopyViz.cpp)

include_directories(
                   )

target_link_libraries(${LIB} 
                      ${PROJECT_NAME}_homotopy
                      ${QT_LIBRARIES} 
                      ${OpenCV_LIBRARIES}
                     )

install(TARGETS ${LIB}
   LIBRARY
   DESTINATION lib
   ARCHIVE
   DESTINATION lib)

