QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HomotopyViz
TEMPLATE = lib
CONFIG += staticlib

DESTDIR = ../lib

QMAKE_CXXFLAGS += -frounding-math

INCLUDEPATH += /usr/local/include

INCLUDEPATH += /usr/local/include/opencv2
LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_nonfree

INCLUDEPATH += ../Homotopy

LIBS += -L../lib -lHomotopy

SOURCES += \
    homotopyviz.cpp

HEADERS  += \
    homotopyviz.h
