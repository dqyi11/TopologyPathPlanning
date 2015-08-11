QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HomotopyViz
TEMPLATE = lib
CONFIG += staticlib

DESTDIR = ../lib

INCLUDEPATH += ../Homotopy

LIBS += -L../lib -lHomotopy

SOURCES += \

HEADERS  += \

