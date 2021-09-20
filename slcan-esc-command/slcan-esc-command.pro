TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += c++11

SOURCES += \
    main.cpp

HEADERS += \
    ../servosila-common/slcan-encoder.h
