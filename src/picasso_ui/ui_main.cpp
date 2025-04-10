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

#include "testui/mainwindow.h"

#include "picasso_ui.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    auto ui = std::make_shared<PicassoUI>(rclcpp::NodeOptions());
    ui->show();

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(ui);
    executor->spin();

    return app.exec();
}