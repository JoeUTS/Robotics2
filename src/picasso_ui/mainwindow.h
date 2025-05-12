#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <chrono>
#include <memory>
#include <string>

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
#include <QVBoxLayout>
#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ros_image_to_qimage/ros_image_to_qimage.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "picasso_bot/srv/get_image.hpp"
#include "../common/ServiceCommon.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startCamera();
    void connectUR3();
    void captureImage();
    void sendEmergencyStop();
    void previewSketch();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    

 //   void on_widget_customContextMenuRequested(const QPoint &pos);

private:
    Ui::MainWindow *ui;
   // QCamera *camera;
    QVideoWidget *viewfinder;
    QCameraImageCapture *imageCapture;
    QLabel *imageLabel_;
    QLabel *sketchLabel_;
    bool eyesShutdownComplete_; // Flag to check if eyes module is shutdown during closing
    

    // ROS 2
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servCamerafeed_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servEyesShutdown_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servCaptureImage_;
    rclcpp::Client<picasso_bot::srv::GetImage>::SharedPtr servPreviewSketch_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servDiscardImage_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servGenerateToolpath_; 

    sensor_msgs::msg::Image sketchMsg_;
    
    void serviceSketchRequest(void);
    void serviceSketchRespose(rclcpp::Client<picasso_bot::srv::GetImage>::SharedFuture future);

    void serviceShutdownEyesRequest(void);
    void serviceShutdownEyesRespose(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

};
#endif // MAINWINDOW_H
