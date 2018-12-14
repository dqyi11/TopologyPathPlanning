set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(LIB ${PROJECT_NAME}_homotopy_viz)

add_library(${LIB} SHARED
            ${PROJECT_SOURCE_DIR}/include/topologyPathPlanning/homotopyviz/HomotopyViz.hpp
            HomotopyViz.cpp)

qt5_use_modules(${LIB} Widgets)

target_link_libraries(${LIB} 
                      ${PROJECT_NAME}_homotopy
                      ${QT_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                     )

#install(TARGETS ${LIB}
#   LIBRARY
#   DESTINATION lib
#   ARCHIVE
#   DESTINATION lib)

