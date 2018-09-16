set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(${PROJECT_NAME}_spatial_infer_demo
               SpatialInferWindow.cpp
               SpatialInferConfig.cpp
               SpatialInferDemo.cpp) 

include_directories(
                   )

target_link_libraries(${PROJECT_NAME}_spatial_infer_demo 
                      ${PROJECT_NAME}_spatial_infer
                      ${PROJECT_NAME}_spatial_infer_viz
                      ${QT_LIBRARIES}
                     ) 

