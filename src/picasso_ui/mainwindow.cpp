#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Initialize ROS 2
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    // Create a node and publishers
    node = rclcpp::Node::make_shared("image_publisher_node");
    image_publisher = node->create_publisher<sensor_msgs::msg::Image>("/processed_camera_image", 10);
    image_subscriber = node->create_subscription<sensor_msgs::msg::Image>(
        "/processed_camera_image", 10,
        std::bind(&MainWindow::imageCallback, this, std::placeholders::_1));

    // Generate a blue image and start publishing it (Placeholder code for camera)
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 0, 0)); // Blue image
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    auto loop_rate = std::make_shared<rclcpp::Rate>(30); // 30 Hz
    std::thread([this, image_msg, loop_rate]() mutable {
        while (rclcpp::ok()) {
            image_publisher->publish(*image_msg);
            rclcpp::spin_some(node);
            loop_rate->sleep();
        }
    }).detach();

    // Initialize the PicassoEyes node
    //picassoEyesNode = new PicassoEyes();

    // Connect button
    connect(ui->startCamera, &QPushButton::clicked, this, &MainWindow::startCamera);
    connect(ui->captureImage, &QPushButton::clicked, this, &MainWindow::captureImage);
    connect(ui->connectUR3, &QPushButton::clicked, this, &MainWindow::connectUR3);
    connect(ui->previewSketch, &QPushButton::clicked, this, &MainWindow::previewSketch);
    // connect(ui->eStopButton, &QPushButton::clicked, this, &MainWindow::sendEmergencyStop);
}

MainWindow::~MainWindow() {
    //delete picassoEyesNode;
    rclcpp::shutdown();
    delete ui;
}

void MainWindow::startCamera() {

    // Start the picasso_eyes launch file
    QString command = "ros2 launch picasso_eyes realsense.launch.py";
    QProcess *process = new QProcess(this);
    QStringList arguments = command.split(' ', Qt::SkipEmptyParts);
    QString program = arguments.takeFirst();
    process->start(program, arguments);

    // Ensure the ROS 2 subscription is active
    RCLCPP_INFO(node->get_logger(), "Starting camera and initializing ROS image view...");

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

    // Display a message indicating the camera has started
    RCLCPP_INFO(node->get_logger(), "Camera started. Waiting for images...");
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
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

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
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MainWindow::previewSketch() {
    // Ensure the PicassoEyes node is initialized
    //if (!picassoEyesNode) {
        //RCLCPP_ERROR(node->get_logger(), "PicassoEyes node is not initialized.");
        //return;
    //}

    // Use a public function in PicassoEyes to retrieve the processed image
    //cv::Mat sketch = picassoEyesNode->getSketchPreview();  // Hypothetical public function in PicassoEyes

    //if (sketch.empty()) {
        //RCLCPP_ERROR(node->get_logger(), "No sketch preview available.");
        //return;
    //}

    // Convert the sketch to QImage and display it in the QLabel
    //QImage qSketch(sketch.data, sketch.cols, sketch.rows, sketch.step, QImage::Format_BGR888);
    //QLabel *imageLabel = ui->viewfinderPlaceholder->findChild<QLabel *>("imageLabel");
    //if (imageLabel) {
        //imageLabel->setPixmap(QPixmap::fromImage(qSketch));
    //} else {
        //RCLCPP_ERROR(node->get_logger(), "Image label not found.");
    //}
}