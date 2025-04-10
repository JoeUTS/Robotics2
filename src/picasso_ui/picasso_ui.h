#ifndef PICASSOUI_H
#define PICASSOUI_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QMainWindow>
#include <QObject>
#include <QPushButton>
#include "ui_test.h"


class PicassoUI : public QMainWindow, public rclcpp::Node {
    Q_OBJECT

    public:
        PicassoUI(rclcpp::NodeOptions options);

    private slots:
        void onPushButtonClicked();

    
    private:
        std::unique_ptr<Ui::MainWindow> ui_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif // PICASSOUI_H