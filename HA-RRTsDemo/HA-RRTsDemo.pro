QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HA-RRTs-Demo
TEMPLATE = app

DESTDIR = ../bin

INCLUDEPATH += ../Homotopy \
               ../HA-RRTs \
               ../HA-RRTsViz

LIBS += -L../lib -lHomotopy -lHA-RRTs -lHA-RRTsViz

SOURCES += \

HEADERS  += \
