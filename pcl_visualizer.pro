#-------------------------------------------------
#
# Project created by QtCreator 2014-11-11T14:00:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcd_viewer
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    colordialog.cpp \
    triangulationdialog.cpp

HEADERS  += pclviewer.h \
    colordialog.h \
    triangulationdialog.h

FORMS    += pclviewer.ui \
    colordialog.ui \
    triangulationdialog.ui

RESOURCES += \
    resources.qrc


