
set(LIB homotopy-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#qt4_wrap_cpp(homotopy-viz_hdr_moc homotopyviz.h)

add_library(${LIB} SHARED
            homotopyviz.h
            homotopyviz.cc)
#            ${homotopy-viz_hdr_moc})

target_link_libraries(${LIB} homotopy ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${OpenCV_LIBRARIES})

