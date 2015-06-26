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
    colordialog.cpp \
    triangulationdialog.cpp \
    compute_triangulation_mesh.cpp \
    triangulation_meshes.cpp \
    tourdialog.cpp \
    boundingbox.cpp \
    worker.cpp \
    setcameradialog.cpp \
    datamodel.cpp \
    pcdviewermainwindow.cpp \
    myqvtkwidget.cpp

HEADERS  += \
    colordialog.h \
    triangulationdialog.h \
    triangulation_meshes.h \
    tourdialog.h \
    boundingbox.h \
    worker.h \
    setcameradialog.h \
    datamodel.h \
    pcdviewermainwindow.h \
    myqvtkwidget.h

FORMS    += \
    colordialog.ui \
    triangulationdialog.ui \
    setcameradialog.ui \
    tourdialog.ui \
    pcdviewermainwindow.ui

RESOURCES += \
    resources.qrc

DISTFILES += \
    CMakeLists.txt


