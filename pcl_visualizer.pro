#-------------------------------------------------
#
# Project created by QtCreator 2014-11-11T14:00:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = colorize_cloud
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    colordialog.cpp

HEADERS  += pclviewer.h \
    colordialog.h

FORMS    += pclviewer.ui \
    colordialog.ui

RESOURCES += \
    resources.qrc


