#-------------------------------------------------
#
# Project created by QtCreator 2016-07-31T15:39:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = kinect2_viewer
TEMPLATE = app


SOURCES += \
    MainUI.cpp \
    Main.cpp \
    ViewerWindow.cpp \
    DataProcess.cpp \
    PointCloudIO.cpp \
    PointCloudProcess.cpp \
    TextFileIO.cpp

HEADERS  += \
    MainUI.h \
    ViewerWindow.h \
    DataProcess.h \
    PointCloudIO.h \
    PointCloudProcess.h \
    TextFileIO.h

FORMS    += \
    MainUI.ui
