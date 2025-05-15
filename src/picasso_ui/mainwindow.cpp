#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), Node("picaso_ui"), ui(new Ui::MainWindow)
{
    // Qt
    // widgets dont scale with window size + too small for containing words
    ui->setupUi(this);

    imageLabel_ = new QLabel(ui->viewfinderPlaceholder);
    imageLabel_->setObjectName("imageLabel");
    imageLabel_->setScaledContents(true);
    
    QVBoxLayout *layoutImage = qobject_cast<QVBoxLayout *>(ui->viewfinderPlaceholder->layout());
    if (!layoutImage) {
        layoutImage = new QVBoxLayout(ui->viewfinderPlaceholder);
        ui->viewfinderPlaceholder->setLayout(layoutImage);
    }
    layoutImage->addWidget(imageLabel_);

    sketchLabel_ = new QLabel(ui->viewfinderPlaceholder_2);
    sketchLabel_->setObjectName("sketchLabel");
    sketchLabel_->setScaledContents(true);

    QVBoxLayout *layoutSketch = qobject_cast<QVBoxLayout *>(ui->viewfinderPlaceholder_2->layout());
    if (!layoutSketch) {
        layoutSketch = new QVBoxLayout(ui->viewfinderPlaceholder_2);
        ui->viewfinderPlaceholder_2->setLayout(layoutSketch);
    }
    layoutSketch->addWidget(sketchLabel_);

    connect(ui->startCamera, &QPushButton::clicked, this, &MainWindow::startCamera);
    connect(ui->captureImage, &QPushButton::clicked, this, &MainWindow::captureImage);
    connect(ui->connectUR3, &QPushButton::clicked, this, &MainWindow::connectUR3);
    connect(ui->previewSketch, &QPushButton::clicked, this, &MainWindow::previewSketch);
    connect(ui->discardImage , &QPushButton::clicked, this, &MainWindow::discardImage);
    connect(ui->generateToolpath , &QPushButton::clicked, this, &MainWindow::generateToolpath);
    connect(ui->startDrawing , &QPushButton::clicked, this, &MainWindow::startDrawing);
    // connect(ui->eStopButton, &QPushButton::clicked, this, &MainWindow::sendEmergencyStop);


    // ROS
    eyesShutdownComplete_ = false;
    sketchMsg_ = sensor_msgs::msg::Image();

    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/processed_camera_image", 10,
        std::bind(&MainWindow::imageCallback, this, std::placeholders::_1));

    servCamerafeed_ = this->create_client<std_srvs::srv::Trigger>("/camera_feed_toggle");
    servEyesShutdown_ = this->create_client<std_srvs::srv::Trigger>("/shutdown_node");
    servCaptureImage_ = this->create_client<std_srvs::srv::Trigger>("/capture_image");
    servPreviewSketch_ = this->create_client<picasso_bot::srv::GetImage>("/preview_sketch");
    servDiscardImage_ = this->create_client<std_srvs::srv::Trigger>("/discard_image");          // TO DO: add button
    servGenerateToolpath_ = this->create_client<std_srvs::srv::Trigger>("/generate_toolpath");  // TO DO: add button
    servStartDrawing_ = this->create_client<std_srvs::srv::Trigger>("/start_drawing");
    servStopDrawing_ = this->create_client<std_srvs::srv::Trigger>("/stop_drawing");
    servHomePose_ = this->create_client<std_srvs::srv::Trigger>("/home_pose");
    servEStop_ = this->create_client<std_srvs::srv::Trigger>("/e_stop");
}

MainWindow::~MainWindow() {
    // Shutdown eyes module
    serviceShutdownEyesRequest();
    while (!eyesShutdownComplete_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Shutdown arm module
    // TO DO

    rclcpp::shutdown();

    delete ui;
}


void MainWindow::startCamera() {
    serviceRequest<std_srvs::srv::Trigger>(servCamerafeed_, this->shared_from_this());

    RCLCPP_INFO(this->get_logger(), "Camera started. Waiting for images..."); 
}

void MainWindow::connectUR3() {
    QString command = "ros2 launch picasso_arms ur3.launch.py";
    QProcess *process = new QProcess(this);
    process->start(command);
}


void MainWindow::captureImage() {
    serviceRequest<std_srvs::srv::Trigger>(servCaptureImage_, this->shared_from_this());
}


void MainWindow::sendEmergencyStop() {
   std_msgs::msg::Bool msg;
   msg.data = true;
   estop_publisher->publish(msg);

   serviceRequest<std_srvs::srv::Trigger>(servEStop_, this->shared_from_this());
}


void MainWindow::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Changed to simplify, fix dangling pointer and use QLabel member variable - Joseph
    if (msg->width == 0 && msg->height == 0) {
        return;
    }

    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    QImage tempQImage(image.data, image.cols, image.rows, image.step, QImage::Format_BGR888);
    QImage safeQImage = tempQImage.copy();  // Need to have a copy to avoid dangling pointer

    auto weak_ptr_this = std::weak_ptr<MainWindow>(
            std::static_pointer_cast<MainWindow>(this->shared_from_this()));

    QMetaObject::invokeMethod(this, [weak_ptr_this, qImage = std::move(safeQImage)]() {
        auto shared_this = weak_ptr_this.lock();

        if (!shared_this) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot update imageLabel_.");
            rclcpp::shutdown();
        }

        shared_this->imageLabel_->setPixmap(QPixmap::fromImage(qImage));
    });
}

void MainWindow::previewSketch() {
    // CHanged to fix dangling pointer and use QLabel member variable - Joseph
    serviceSketchRequest();

    while (sketchMsg_ == sensor_msgs::msg::Image()) {   // Wait for sketch to be received
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(sketchMsg_, "mono8");
    cv::Mat sketch = cvPtr->image;

    if (sketch.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No sketch preview available.");
        return;
    }

    QImage tempQSketch(sketch.data, sketch.cols, sketch.rows, sketch.step, QImage::Format_Grayscale8);
    QImage safeQSketch = tempQSketch.copy();

    auto weakThis = std::weak_ptr<MainWindow>(
            std::static_pointer_cast<MainWindow>(this->shared_from_this()));

    QMetaObject::invokeMethod(this, [weakThis, qImage = std::move(safeQSketch)]() {
        auto sharedThis = weakThis.lock();

        if (!sharedThis) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot update sketchLabel_.");
            rclcpp::shutdown();
        }

        sharedThis->sketchLabel_->setPixmap(QPixmap::fromImage(qImage));
    });
}

void MainWindow::serviceSketchRequest(void) {
    serviceWait<picasso_bot::srv::GetImage>(servPreviewSketch_);

    sketchMsg_ = sensor_msgs::msg::Image();

    auto request = std::make_shared<picasso_bot::srv::GetImage::Request>();
    auto weak_this = std::weak_ptr<MainWindow>(
        std::static_pointer_cast<MainWindow>(this->shared_from_this()));

    servPreviewSketch_->async_send_request(request,
        [weak_this, this](rclcpp::Client<picasso_bot::srv::GetImage>::SharedFuture future) {

        auto shared_this = weak_this.lock();
        if (shared_this) {
            shared_this->serviceSketchRespose(future);

        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot process service '%s'.", std::string(servPreviewSketch_->get_service_name()));
        }
    });

    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", std::string(servPreviewSketch_->get_service_name()));
}

void MainWindow::serviceSketchRespose(rclcpp::Client<picasso_bot::srv::GetImage>::SharedFuture future) {
    auto result = future.get();
    if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Service '%s' called successfully.", std::string(servPreviewSketch_->get_service_name()));
        sketchMsg_ = result->image;

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed.", std::string(servPreviewSketch_->get_service_name()));
    }
}

void MainWindow::serviceShutdownEyesRequest(void) {
    serviceWait<std_srvs::srv::Trigger>(servEyesShutdown_);
    
    eyesShutdownComplete_ = false;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto weak_this = std::weak_ptr<MainWindow>(
        std::static_pointer_cast<MainWindow>(this->shared_from_this()));

    servEyesShutdown_->async_send_request(request,
        [weak_this, this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            
        auto shared_this = weak_this.lock();
        if (shared_this) {
            shared_this->serviceShutdownEyesRespose(future);

        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot process service '%s'.", std::string(servEyesShutdown_->get_service_name()));
        }
    });

    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", std::string(servEyesShutdown_->get_service_name()));
}

void MainWindow::serviceShutdownEyesRespose(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    auto result = future.get();
    if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Service '%s' called successfully.", std::string(servEyesShutdown_->get_service_name()));
        eyesShutdownComplete_ = true;

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed: %s", std::string(servEyesShutdown_->get_service_name()), result->message);
    }
}

void MainWindow::discardImage() {
    serviceRequest<std_srvs::srv::Trigger>(servDiscardImage_, this->shared_from_this());
}

void MainWindow::generateToolpath() {
    serviceRequest<std_srvs::srv::Trigger>(servGenerateToolpath_, this->shared_from_this());
}

void MainWindow::startDrawing() {
    serviceRequest<std_srvs::srv::Trigger>(servStartDrawing_, this->shared_from_this());
}

void MainWindow::stopDrawing() {
    serviceRequest<std_srvs::srv::Trigger>(servStopDrawing_, this->shared_from_this());
}