
set(LIB ${PROJECT_NAME}_spatial_infer_viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            ${PROJECT_SOURCE_DIR}/include/topologyPathPlanning/spatialinferviz/SpatialInferViz.hpp
            SpatialInferViz.cpp)

include_directories(
                   )

target_link_libraries(${LIB} 
                      ${PROJECT_NAME}_homotopy
                      ${PROJECT_NAME}_spatial_infer
                      ${QT_LIBRARIES} 
                      ${OpenCV_LIBRARIES}
                     )


