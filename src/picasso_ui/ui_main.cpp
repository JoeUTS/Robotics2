#include "mainwindow.h"

#include <QApplication>

#include <memory>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);

    auto main_window_node = std::make_shared<MainWindow>();
    std::thread ros_spin_thread([main_window_node]() {
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(main_window_node);
        exec.spin();
    });

    main_window_node->show();

    // Cleanup
    int ret = a.exec();
    rclcpp::shutdown();
    ros_spin_thread.join();
    return ret;

    //MainWindow w;
    //w.show();
    //return a.exec();
}
