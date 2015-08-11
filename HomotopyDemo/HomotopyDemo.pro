QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Homotopy-Demo
TEMPLATE = app

DESTDIR = ../bin

INCLUDEPATH += ../Homotopy \
               ../HomotopyViz

LIBS += -L../lib -lHomotopy -lHomotopyViz

SOURCES += \

HEADERS  += \
