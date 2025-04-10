#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include <QtMultimedia/QtMultimedia>
#include <QtMultimediaWidgets/QtMultimediaWidgets>
#include <QCamera>
#include <QCameraImageCapture>
#include <QCameraViewfinder>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "ros_image_to_qimage/ros_image_to_qimage.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startCamera();
    void captureImage();
    void sendEmergencyStop();

 //   void on_widget_customContextMenuRequested(const QPoint &pos);

private:
    Ui::MainWindow *ui;
   // QCamera *camera;
    QVideoWidget *viewfinder;
    QCameraImageCapture *imageCapture;

    // ROS 2
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    //auto node = rclcpp::Node::make_shared("image_publisher_node");
   // auto publisher = node->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);

};
#endif // MAINWINDOW_H
