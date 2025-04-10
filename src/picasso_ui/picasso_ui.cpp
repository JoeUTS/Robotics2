// #include "picasso_ui.h"

// PicassoUI::PicassoUI(void) : Node("picasso_ui") {

// }

#include "picasso_ui.h"


PicassoUI::PicassoUI(rclcpp::NodeOptions options)
    : QMainWindow(), rclcpp::Node("picasso_ui", options)

{
    ui = new Ui::MainWindow();  // Create the UI
    ui->setupUi(this);  // Set up the UI

    // Initialize the ROS publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("command_topic", 10);

    // Connect the push button signal to the slot
    connect(ui->pushButton, &QPushButton::clicked, this, &PicassoUI::onPushButtonClicked);
}

PicassoUI::~PicassoUI() {
    delete ui;  // Clean up UI resources
}

void PicassoUI::onPushButtonClicked()
{
    auto message = std_msgs::msg::String();
    message.data = "Command sent!";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sent command: '%s'", message.data.c_str());
}