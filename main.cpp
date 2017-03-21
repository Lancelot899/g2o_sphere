#include <QApplication>
#include <glog/logging.h>

#include "MainWidget.h"
#include <QGLWidget>


int main(int argc, char *argv[]) {
    QApplication app(argc, argv); //! 初始化一个qt的运行实例
    google::InitGoogleLogging(argv[0]);

    MainWidget w; //! 初始化一个窗口类
    w.show(); //! 显示窗口

    return app.exec();   //! 这个是运行qt的消息循环

}

