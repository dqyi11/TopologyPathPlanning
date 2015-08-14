QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Homotopy-Demo
TEMPLATE = app

DESTDIR = ../bin

QMAKE_CXXFLAGS += -frounding-math

INCLUDEPATH += ../Homotopy \
               ../HomotopyViz

LIBS += -L../lib -lHomotopy -lHomotopyViz -lCGAL

SOURCES += \
    mainwindow.cpp \
    main.cpp

HEADERS  += \
    mainwindow.h
