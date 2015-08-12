TEMPLATE = lib
TARGET = Homotopy
CONFIG += staticlib

DESTDIR = ../lib

win32{

INCLUDEPATH += C:\\CGAL-4.6.1\include

}


LIBS += -L/usr/lib -lCGAL


SOURCES += \
    worldmap.cpp \
    obstacle.cpp \
    region.cpp

HEADERS  += \
    worldmap.h \
    obstacle.h \
    world_datatype.h \
    region.h

