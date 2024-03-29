#-------------------------------------------------
#
# Project created by QtCreator 2012-11-19T14:37:34
#
#-------------------------------------------------

QT       += core gui

TARGET = qtOpenCV
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    placedetector.cpp \
    cvarOpenCVDetector.cpp \
    cvarOpenCVFrameHolder.cpp

HEADERS  += mainwindow.h \
    placedetector.h \
    cvarOpenCVDetector.h \
    cvarOpenCVFrameHolder.h \
    cvarDetectInfo.h

FORMS    += mainwindow.ui

RESOURCES     = application.qrc \
    application.qrc

unix {
   CONFIG += link_pkgconfig
   PKGCONFIG += opencv
}
