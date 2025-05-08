#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), Node("picaso_ui"), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Initialize ROS 2
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    // Create a node and publishers
    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/processed_camera_image", 10,
        std::bind(&MainWindow::imageCallback, this, std::placeholders::_1));

    servCamerafeed_ = this->create_client<std_srvs::srv::Trigger>("/camera_feed_toggle");
    servEyesShutdown_ = this->create_client<std_srvs::srv::Trigger>("/picasso_eyes/shutdown_node");

    // Connect button
    connect(ui->startCamera, &QPushButton::clicked, this, &MainWindow::startCamera);
    connect(ui->captureImage, &QPushButton::clicked, this, &MainWindow::captureImage);
    connect(ui->connectUR3, &QPushButton::clicked, this, &MainWindow::connectUR3);
    connect(ui->previewSketch, &QPushButton::clicked, this, &MainWindow::previewSketch);
    // connect(ui->eStopButton, &QPushButton::clicked, this, &MainWindow::sendEmergencyStop);
}

MainWindow::~MainWindow() {
    shutdownEyes();
    rclcpp::shutdown(); // Make sure this is the last command in the destructor - Joseph
}

void MainWindow::startCamera() {

    toggleCameraFeed();
    // Now that I have made a combined launch file, this probs isnt needed sorry.
    // We now have a camera toggle function at the bottom which can toggle the camera publishing.
    //- Joseph

    // // Start the picasso_eyes launch file
    // QString command = "ros2 launch picasso_eyes realsense.launch.py";
    // QProcess *process = new QProcess(this);
    // QStringList arguments = command.split(' ', Qt::SkipEmptyParts);
    // QString program = arguments.takeFirst();
    // process->start(program, arguments);

    // // Ensure the ROS 2 subscription is active
    // RCLCPP_INFO(this->get_logger(), "Starting camera and initializing ROS image view...");

    // // Find or create the QLabel in the existing layout
    QLabel *imageLabel = ui->viewfinderPlaceholder->findChild<QLabel *>("imageLabel");

    if (!imageLabel) {
        // If QLabel doesn't exist, create it and add it to the layout
        imageLabel = new QLabel(ui->viewfinderPlaceholder);
        imageLabel->setObjectName("imageLabel");
        imageLabel->setScaledContents(true); // Scale the image to fit the label

        // Add QLabel to the existing layout
        QVBoxLayout *layout = qobject_cast<QVBoxLayout *>(ui->viewfinderPlaceholder->layout());
        if (!layout) {
            layout = new QVBoxLayout(ui->viewfinderPlaceholder);
            ui->viewfinderPlaceholder->setLayout(layout);
        }
        layout->addWidget(imageLabel);
    }

    // Display a message indicating the camera has started
    RCLCPP_INFO(this->get_logger(), "Camera started. Waiting for images..."); 
}

void MainWindow::connectUR3() {
    QString command = "ros2 launch picasso_arms ur3.launch.py";
    QProcess *process = new QProcess(this);
    process->start(command);
}


void MainWindow::captureImage() {
    // Get the current image from the QLabel
    QLabel *imageLabel = ui->viewfinderPlaceholder->findChild<QLabel *>("imageLabel");
    if (imageLabel) {
        QPixmap pixmap = *(imageLabel->pixmap());
        if (!pixmap.isNull()) {
            // Save the image to a file
            QString filePath = QFileDialog::getSaveFileName(this, "Save Image", QDir::homePath(), "Images (*.jpg *.png *.bmp)");
            if (!filePath.isEmpty()) {
                pixmap.toImage().save(filePath);
            }
        }
    }
}

void MainWindow::sendEmergencyStop() {
   std_msgs::msg::Bool msg;
   msg.data = true;
   estop_publisher->publish(msg);
}


void MainWindow::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // Convert ROS 2 Image message to OpenCV image
       cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image; //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% change this placeholder

        // Convert OpenCV image to QImage
        QImage qImage(image.data, image.cols, image.rows, image.step, QImage::Format_BGR888);

        // Use QMetaObject::invokeMethod to update the GUI in the main thread
        QMetaObject::invokeMethod(this, [this, qImage]() {
            // Find or create the QLabel in the existing layout
            QLabel *imageLabel = ui->viewfinderPlaceholder->findChild<QLabel *>("imageLabel");

            if (!imageLabel) {
                // If QLabel doesn't exist, create it and add it to the layout
                imageLabel = new QLabel(ui->viewfinderPlaceholder);
                imageLabel->setObjectName("imageLabel");
                imageLabel->setScaledContents(true); // Scale the image to fit the label

                // Add QLabel to the existing layout
                QVBoxLayout *layout = qobject_cast<QVBoxLayout *>(ui->viewfinderPlaceholder->layout());
                if (!layout) {
                    layout = new QVBoxLayout(ui->viewfinderPlaceholder);
                    ui->viewfinderPlaceholder->setLayout(layout);
                }
                layout->addWidget(imageLabel);
            }

            // Update the QLabel with the new image
            imageLabel->setPixmap(QPixmap::fromImage(qImage));
        });
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MainWindow::previewSketch() {
    /*
    if (sketch.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No sketch preview available.");
        return;
    }

    // Convert the sketch to QImage and display it in the QLabel
    QImage qSketch(sketch.dat    QImage qSketch(sketch.data, sketch.cols, sketch.rows, sketch.step, QImage::Format_BGR888);
    QLabel *imageLabel = ui->viewfinderPlaceholder->findChild<QLabel *>("imageLabel");

 
    if (imageLabel) {
        imageLabel->setPixmap(QPixmap::fromImage(qSketch));
    } else {
        RCLCPP_ERROR(node->get_logger(), "Image label not found.");
    }a, sketch.cols, sketch.rows, sketch.step, QImage::Format_BGR888);*/

    cv::Mat sketch = ;//code from picassoeyes
    if (sketch.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No sketch preview available.");
        return;
    }

    QImage qSketch(sketch.data, sketch.cols, sketch.rows, sketch.step, QImage::Format_BGR888);
    QLabel *sketchLabel = ui->previewSketch->findChild<QLabel *>("sketchLabel");

    if (sketchLabel) {
        sketchLabel->setPixmap(QPixmap::fromImage(qSketch));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Image label not found.");
    }
    
}

bool MainWindow::toggleCameraFeed(void) {
    auto messagePeriod = std::chrono::seconds(1);
    std::chrono::time_point<std::chrono::system_clock> lastMsg;
    
    // Wait for service
    while (!servCamerafeed_ ->wait_for_service(std::chrono::milliseconds(250))) {
        // Prevent spaming messages
        std::chrono::duration<double> timeSinceLastMsg = std::chrono::system_clock::now() - lastMsg;
        if (timeSinceLastMsg >= messagePeriod) {
            lastMsg = std::chrono::system_clock::now();
            RCLCPP_INFO_STREAM(this->get_logger(), "waiting for service 'camera_feed_toggle' to connect");
        }
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = servCamerafeed_->async_send_request(request);
    bool success = false;

    // Await responce
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Camera toggled.");
          success = true;

        } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Camera toggle failed.");
        }

      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call service 'camera_feed_toggle'");
    }

    return success;
}

void MainWindow::shutdownEyes(void) {
    auto messagePeriod = std::chrono::seconds(1);
    std::chrono::time_point<std::chrono::system_clock> lastMsg;
    
    // Wait for service
    while (!servEyesShutdown_ ->wait_for_service(std::chrono::milliseconds(250))) {
        // Prevent spaming messages
        std::chrono::duration<double> timeSinceLastMsg = std::chrono::system_clock::now() - lastMsg;
        if (timeSinceLastMsg >= messagePeriod) {
            lastMsg = std::chrono::system_clock::now();
            RCLCPP_INFO_STREAM(this->get_logger(), "waiting for service 'picasso_eyes/shutdown_node' to connect");
        }
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = servCamerafeed_->async_send_request(request);
    bool success = result.get()->success;
}