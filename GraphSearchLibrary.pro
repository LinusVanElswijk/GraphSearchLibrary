#-------------------------------------------------
#
# Project created by QtCreator 2013-06-19T12:26:24
#
#-------------------------------------------------

QT       -= core gui

TARGET = GraphSearchLibrary
TEMPLATE = lib
CONFIG += staticlib
CONFIG += c++11

SOURCES +=

HEADERS += \
    Problem.hpp \
    GridMapProblem.hpp \
    SearchAlgorithm.hpp \
    Solution.hpp \
    NodeListeners.hpp \
    Node.hpp \
    Fringe.hpp \
    AStarFringe.hpp \
    AStar.hpp \
    HeuristicFunction.hpp \
    FifoFringe.hpp \
    BreadthFirstSearch.h \
    ThetaStar.hpp \
    LineOfSightFunction.hpp
unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

OTHER_FILES += \
    Problem \
    GridMapProblem \
    GraphSearchLibrary.pro.user
