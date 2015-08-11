TEMPLATE = lib
TARGET = Homotopy
CONFIG += staticlib

DESTDIR = ../lib

INCLUDEPATH += /usr/local/include/opencv

LIBS += -L/usr/local/lib \
        -lopencv_core

SOURCES += \
    worldmap.cpp \
    obstacle.cpp

HEADERS  += \
    worldmap.h \
    utility.h \
    obstacle.h \
    world_datatype.h

