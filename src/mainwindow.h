#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void print(QString s);
    void set_node_handle(rclcpp::Node::SharedPtr nh);

private slots:

    void on_topicSet_clicked();

    void on_zeroButton_clicked();

    void timer_callback();

private:
    Ui::MainWindow *ui;
    QString topic;
    rclcpp::Node::SharedPtr node_handle; 
    QTimer* publish_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    float min_linear_velocity, max_linear_velocity;
    float min_angular_velocity, max_angular_veloity;
    int publish_rate_ms;
};
#endif // MAINWINDOW_H
