// #include <memory>
// #include <rclcpp/rclcpp.hpp>

// #include "picasso_ui.h"

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PicassoUI>());
//     rclcpp::shutdown();

//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "picasso_ui.h" // Adjust the path based on the actual location of PicassoUI.h

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    auto ui = std::make_shared<PicassoUI>(rclcpp::NodeOptions());
    ui->show();

    rclcpp::spin(std::make_shared<PicassoUI>());
    rclcpp::shutdown();

    return app.exec();
}