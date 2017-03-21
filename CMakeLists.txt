cmake_minimum_required(VERSION 2.8)
project(sphere)

set(CMAKE_CXX_STANDARD 11)


IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++0x")
ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()


INCLUDE(ExternalProject)
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

FIND_PACKAGE(QGLViewer REQUIRED QUIET)
MESSAGE("-- find QGLViewer")
FIND_PACKAGE(SuiteSparse REQUIRED)
MESSAGE("-- find SuiteSparse")
FIND_PACKAGE(Glog REQUIRED QUIET)
MESSAGE("-- find Glog")
FIND_PACKAGE(Cholmod REQUIRED QUIET)
MESSAGE("-- find Cholmod")
FIND_LIBRARY(GOMP_LIB
        NAMES gomp libgomp.so.1
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )
FIND_PACKAGE(BLAS REQUIRED QUIET)
MESSAGE("-- find blas")
FIND_PACKAGE(LAPACK REQUIRED QUIET)
MESSAGE("-- find lapack")
FIND_PACKAGE(Ceres REQUIRED QUIET)
MESSAGE("-- find Ceres")
FIND_PACKAGE(FREEGLUT REQUIRED QUIET)
MESSAGE("-- find freeglut")
FIND_PACKAGE(GLEW REQUIRED QUIET)
MESSAGE("-- find glew")

FIND_LIBRARY(GFLAGS_LIB
        NAMES flags libgflags.so
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )

FIND_LIBRARY(GL_LIB
        NAMES GL gl
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )

IF(GL_LIB)
    MESSAGE("-- find opengl")
ENDIF()


FIND_LIBRARY(
        GLU_LIB
        NAMES glu GLu GlU GLU
        PATH
            /usr/lib/x86_64-linux-gnu
            /opt/local/lib
            /usr/lib
            /usr/local/lib
)

IF(GLU_LIB)
    MESSAGE("-- find glu")
ENDIF()

set(SOURCE_FILES
        sphereScene.cpp
        sphereScene.h
        DataStruct.h
        main.cpp
        MainWidget.cpp
        MainWidget.h
        Sphere.cpp
        Sphere.h
        )


if(Qt4_FOUND)
    INCLUDE(${QT_USE_FILE})

    QT4_WRAP_CPP(UI_SOURCES sphereScene.h MainWidget.h)

    INCLUDE_DIRECTORIES(${QT_INCLUDES})
    SET(MY_QT_LIBRARIES ${QT_LIBRARIES})
endif()

if(Qt5_FOUND)
    QT5_WRAP_CPP(UI_SOURCES sphereScene.h MainWidget.h)

    INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
    SET(MY_QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
endif()


INCLUDE_DIRECTORIES(
        .
        ./Thirdparty
        ${GLOG_INCLUDE_DIR}
        ${CERES_INCLUDE_DIR}
        ${SuiteSparse_INCLUDE_DIRS}
        ${QGLVIEWER_INCLUDE_DIR}
        ${FREEGLUT_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIR}
)



FIND_LIBRARY(GLOG_LIB
        NAMES glog glog.so libglog.so
        PATH
            /usr/lib/x86_64-linux-gnu
            /opt/local/lib
            /usr/lib
            /usr/local/lib
        )

LIST(APPEND LINK_LIBS
        ${GLOG_LIBRARIES}
        ${CERES_LIBRARIES}
        ${QGLVIEWER_LIBRARY_RELEASE}
        ${MY_QT_LIBRARIES}
        ${OPENGL_LIBRARY}
        ${FREEGLUT_LIBRARY}
        ${GLEW_LIBRARY}
        ${GLU_LIB}
        ${GL_LIB}
        ${SUITESPARSE_LIBRARIES}
        ${GOMP_LIB}
        ${CHOLMOD_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${GFLAGS_LIB}
        ${GLOG_LIB}
        )


add_executable(sphere ${SOURCE_FILES} ${UI_SOURCES})
TARGET_LINK_LIBRARIES(sphere ${LINK_LIBS} pthread)
