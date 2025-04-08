// #ifndef PICASSOUI_H
// #define PICASSOUI_H

// #include <rclcpp/rclcpp.hpp>

// class PicassoUI : public rclcpp::Node {
// public:
//   PicassoUI(void);

// private:

// };

// #endif // PICASSOUI_H

#ifndef PICASSOUI_H
#define PICASSOUI_H

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
        PicassoUI();
        ~PicassoUI();

    private slots:
        void onPushButtonClicked();

    
    private:
        Ui::MainWindow *ui;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    
    
};

#endif // PICASSOUI_H