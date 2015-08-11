TEMPLATE = lib
TARGET = Homotopy
CONFIG += staticlib

DESTDIR = ../lib

win32{

INCLUDEPATH += C:\\opencv\build\include \
               C:\\CGAL-4.6.1\include

LIBS += -LC:\opencv\build\x86\vc10\lib \
        -lopencv_core249
}

linux {

INCLUDEPATH += /usr/local/include/opencv

LIBS += -L/usr/local/lib \
        -lopencv_core

}

SOURCES += \
    worldmap.cpp \
    obstacle.cpp

HEADERS  += \
    worldmap.h \
    utility.h \
    obstacle.h \
    world_datatype.h

