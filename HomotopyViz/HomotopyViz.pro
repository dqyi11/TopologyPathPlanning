QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HomotopyViz
TEMPLATE = lib
CONFIG += staticlib

DESTDIR = ../lib

win32{

INCLUDEPATH += C:\\opencv\build\include

LIBS += -LC:\opencv\build\x86\vc10\lib \
        -lopencv_core249
}

linux {

INCLUDEPATH += /usr/local/include/opencv

LIBS += -L/usr/local/lib \
        -lopencv_core

}

INCLUDEPATH += ../Homotopy

LIBS += -L../lib -lHomotopy

SOURCES += \
    homotopyviz.cpp

HEADERS  += \
    utility.h \
    homotopyviz.h

