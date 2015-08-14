QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HA-RRTsViz
TEMPLATE = lib
CONFIG += staticlib

DESTDIR = ../lib

INCLUDEPATH += ../HA-RRTs

LIBS += -L../lib -lHA-RRTs

SOURCES += \

HEADERS  += \

