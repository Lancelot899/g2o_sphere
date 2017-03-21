#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <memory>
#include <vector>

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>

class Sphere;

class SphereScene;

class MainWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MainWidget(QWidget *parent = 0);

signals:

public slots:
    void btnInit();                //! 初始化按钮消息相应函数
    void btnOptimize();            //! 优化按钮消息相应函数

private:
    SphereScene  *sphereScene;     //! 球的显示窗口
    QLineEdit    *IterSetEdit;     //! 编辑窗
    QPushButton  *InitButton;      //! 初始化按钮
    QPushButton  *optButton;       //! 优化按钮

private:
    bool  isInit;                  //! 判断是否初始化

private:
    std::shared_ptr<Sphere>  sphere; //! 运算的实例
};

#endif // MAINWIDGET_H
