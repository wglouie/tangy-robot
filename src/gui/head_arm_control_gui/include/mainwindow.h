#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <ros/ros.h>
#include <publisher.h>
#include <QStringList>
#include <QString>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    void setParameters(QStringList max, QStringList min, QStringList reset);
    void setConnections();
    explicit MainWindow(QWidget *parent = 0, ros::NodeHandle *n = NULL);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;
    ros::NodeHandle *nh;
    ros::Publisher lineEdit_pub;
    QLineEdit *lineEdit;
    Publisher *publisher;
    QString head_time;
    QString right_arm_time;
    QString left_arm_time;


private slots:
    void reset_head();
    void reset_right_arm();
    void reset_left_arm();
};

#endif // MAINWINDOW_H
