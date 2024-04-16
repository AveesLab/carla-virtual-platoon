QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    controller.cpp \
    lvthread.cpp \
    fv1thread.cpp \
    fv2thread.cpp \
    ros2node.cpp \

HEADERS += \
    controller.h \
    lvthread.h \
    fv1thread.h \
    fv2thread.h \
    ros2node.hpp \

FORMS += \
    controller.ui

CONFIG += lrelease
CONFIG += embed_translations

INCLUDEPATH += /usr/local/include/opencv4

LIBS += `pkg-config --libs opencv4` \
        -lzmq

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
