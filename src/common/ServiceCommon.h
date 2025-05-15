#ifndef SERVICECOMMON_H
#define SERVICECOMMON_H

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

template <typename ServiceT>
/// \brief Wait for a service to be available
void serviceWait(typename rclcpp::Client<ServiceT>::SharedPtr client) {
    auto messagePeriod = std::chrono::milliseconds(1000);
    std::chrono::time_point<std::chrono::system_clock> lastMsg;
    
    while (!client->wait_for_service(std::chrono::milliseconds(200))) {
        std::chrono::duration<double> duration = std::chrono::system_clock::now() - lastMsg;
        std::chrono::milliseconds timeSinceLastMsg = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
        
        if (timeSinceLastMsg >= messagePeriod) {
        lastMsg = std::chrono::system_clock::now();
        std::string serviceName = std::string(client->get_service_name());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for service '%s' to connect", serviceName.c_str());
        }
    }
}

template <typename ServiceT>
/// \brief Send a service request
void serviceRequest(typename rclcpp::Client<ServiceT>::SharedPtr client,
                    rclcpp::Node::SharedPtr node) {
    serviceWait<ServiceT>(client);

    auto request = std::make_shared<typename ServiceT::Request>();
    auto weak_this = std::weak_ptr<rclcpp::Node>(
        std::static_pointer_cast<rclcpp::Node>(node));

    client->async_send_request(request,
        [weak_this, client, node](typename rclcpp::Client<ServiceT>::SharedFuture future) {

        auto shared_this = weak_this.lock();
        if (shared_this) {
            serviceResponce<ServiceT>(client, future);

        } else {
            std::string serviceName = std::string(client->get_service_name());
            RCLCPP_WARN(node->get_logger(), "Node object expired, cannot process service '%s'.", serviceName.c_str());
        }
    });
    
    std::string serviceName = std::string(client->get_service_name());
    RCLCPP_INFO(node->get_logger(), "Service '%s' request sent.", serviceName.c_str());
}

template <typename ServiceT>
/// \brief Handle the service response
void serviceResponce(typename rclcpp::Client<ServiceT>::SharedPtr client, 
                    typename rclcpp::Client<ServiceT>::SharedFuture future) {
    std::string serviceName = std::string(client->get_service_name());
    auto result = future.get();

    if (result->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service '%s' called successfully.", serviceName.c_str());

    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Service '%s' failed: %s", serviceName.c_str(), result->message);
    }
}

#endif // SERVICECOMMON_H
