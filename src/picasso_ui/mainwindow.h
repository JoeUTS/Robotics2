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
#include <QVBoxLayout>
#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ros_image_to_qimage/ros_image_to_qimage.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow, public rclcpp::Node  // In ROS node should be inherrited.
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void startCamera();
    void connectUR3();
    void captureImage();
    void sendEmergencyStop();
    void previewSketch();
    

 //   void on_widget_customContextMenuRequested(const QPoint &pos);

private:
    Ui::MainWindow *ui;
   // QCamera *camera;
    QVideoWidget *viewfinder;
    QCameraImageCapture *imageCapture;

    // ROS 2
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servCamerafeed_;  // Added service to toggle camera feed - Joseph
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servEyesShutdown_; // Added service to shutdown picasso eyes - Joseph.
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Run this function to set the camera feed to given state - Joseph
    bool toggleCameraFeed(void);

    // Run this function to shutdown eyes - Joseph
    void shutdownEyes(void);
};
#endif // MAINWINDOW_H
