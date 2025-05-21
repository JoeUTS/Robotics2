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
    void discardImage();
    void generateToolpath();
    void startDrawing();
    void stopDrawing();
    

 //   void on_widget_customContextMenuRequested(const QPoint &pos);

private:
    Ui::MainWindow *ui;
   // QCamera *camera;
    QVideoWidget *viewfinder;
    QCameraImageCapture *imageCapture;
    QLabel *imageLabel_;
    QLabel *sketchLabel_;

    // ROS 2
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servCamerafeed_;         // Connect to camera
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servEyesShutdown_;       // Shutdown eyes module
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servCaptureImage_;       // Capture image
    rclcpp::Client<picasso_bot::srv::GetImage>::SharedPtr servPreviewSketch_;  // Preview sketch
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servDiscardImage_;       // Discard captured image
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servGenerateToolpath_;   // Generate toolpath from sketch

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servConnectUR_;          // Connects to UR arm
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servStartDrawing_;       // Starts the drawing process
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servStopDrawing_;        // Stops the drawing process
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servHomePose_;           // Moves the arm to the home pose
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servEStop_;              // E-stop

    sensor_msgs::msg::Image sketchMsg_; // Received sketch
    bool eyesShutdownComplete_;         // Flag to check if eyes module is shutdown during closing
    bool imageCaptured_;                // Flag to check if image is captured
    bool cameraConnected_;              // Flag to check if camera is connected
    bool armConnected_;                 // Flag to check if arm is connected
    std::string serviceLogName_;        // Holds name of service for logging.

    void serviceSketchRequest(void);
    void serviceSketchRespose(rclcpp::Client<picasso_bot::srv::GetImage>::SharedFuture future);
    void serviceShutdownEyesRequest(void);
    void serviceShutdownEyesRespose(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
};
#endif // MAINWINDOW_H