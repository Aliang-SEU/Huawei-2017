TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    cdn/cdn.cpp \
    cdn/deploy.cpp \
    cdn/io.cpp

HEADERS += \
    cdn/deploy.h \
    cdn/lib/lib_io.h \
    cdn/lib/lib_time.h

DISTFILES +=
