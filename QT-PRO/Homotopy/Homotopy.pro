TEMPLATE = lib
TARGET = Homotopy
CONFIG += staticlib

DESTDIR = ../lib

QMAKE_CXXFLAGS += -frounding-math

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

