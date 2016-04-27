QT += core
QT -= gui

TARGET = Boids
CONFIG += console
CONFIG -= app_bundle

INCLUDEPATH +=/usr/local/include/OGRE
INCLUDEPATH += /usr/include/OIS
INCLUDEPATH += /usr/local/include/OGRE/Overlay

#For OpenSteer
LIBS *= -lGLU -lGL -lglut

# For Ogre 1.9
LIBS *= -lOgreOverlay
LIBS *= -lOgreMain -lOIS -lboost_system

TEMPLATE = app

SOURCES += \
    OpenSteer/Camera.cpp \
    OpenSteer/Clock.cpp \
    OpenSteer/Color.cpp \
    OpenSteer/Draw.cpp \
    OpenSteer/Obstacle.cpp \
    OpenSteer/OldPathway.cpp \
    OpenSteer/OpenSteerDemo.cpp \
    OpenSteer/Path.cpp \
    OpenSteer/Pathway.cpp \
    OpenSteer/PlugIn.cpp \
    OpenSteer/PolylineSegmentedPath.cpp \
    OpenSteer/PolylineSegmentedPathwaySegmentRadii.cpp \
    OpenSteer/PolylineSegmentedPathwaySingleRadius.cpp \
    OpenSteer/SegmentedPath.cpp \
    OpenSteer/SegmentedPathway.cpp \
    OpenSteer/TerrainRayTest.cpp \
    OpenSteer/Vec3.cpp \
    OpenSteer/Vec3Utilities.cpp \
    OpenSteer/lq.c \
    BaseApplication.cpp \
    Application.cpp \
    Boids.cpp \
    OpenSteer/SimpleVehicle.cpp \
    metrics.cpp

HEADERS += \
    OpenSteer/AbstractVehicle.h \
    OpenSteer/Annotation.h \
    OpenSteer/Camera.h \
    OpenSteer/Clock.h \
    OpenSteer/Color.h \
    OpenSteer/Draw.h \
    OpenSteer/LocalSpace.h \
    OpenSteer/lq.h \
    OpenSteer/Obstacle.h \
    OpenSteer/OldPathway.h \
    OpenSteer/OpenSteerDemo.h \
    OpenSteer/Path.h \
    OpenSteer/Pathway.h \
    OpenSteer/PlugIn.h \
    OpenSteer/PolylineSegmentedPath.h \
    OpenSteer/PolylineSegmentedPathwaySegmentRadii.h \
    OpenSteer/PolylineSegmentedPathwaySingleRadius.h \
    OpenSteer/Proximity.h \
    OpenSteer/QueryPathAlike.h \
    OpenSteer/QueryPathAlikeBaseDataExtractionPolicies.h \
    OpenSteer/QueryPathAlikeMappings.h \
    OpenSteer/QueryPathAlikeUtilities.h \
    OpenSteer/SegmentedPath.h \
    OpenSteer/SegmentedPathAlikeUtilities.h \
    OpenSteer/SegmentedPathway.h \
    OpenSteer/SharedPointer.h \
    OpenSteer/StandardTypes.h \
    OpenSteer/SteerLibrary.h \
    OpenSteer/TerrainRayTest.h \
    OpenSteer/UnusedParameter.h \
    OpenSteer/Utilities.h \
    OpenSteer/Vec3.h \
    OpenSteer/Vec3Utilities.h \
    BaseApplication.h \
    Application.h \
    Boids.h \
    OpenSteer/SimpleVehicle.h \
    metrics.h

