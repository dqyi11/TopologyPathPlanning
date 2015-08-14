
set(LIB homotopy-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            homotopyviz.h
            homotopyviz.cc)

target_link_libraries(${LIB} homotopy ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${OpenCV_LIBRARIES})

