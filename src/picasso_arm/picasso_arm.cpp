#include "picasso_arm.h"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>


using moveit::planning_interface::MoveGroupInterface;

PicassoArm::PicassoArm(void) : Node("picasso_arm") {
    RCLCPP_INFO(this->get_logger(), "PicassoArm node initialized.");

    // eyes_ = std::make_shared<PicassoEyes>();

    // toolpath_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    //     "/eyes_toolpath", 10,  // <-- replace with actual topic name if different
    //     std::bind(&PicassoArm::toolpath_callback, this, std::placeholders::_1)
    // );
    /*
    timer_ = this->create_wall_timer(
        std::chrono::seconds(5), 
        std::bind(&PicassoArm::moveToPose, this) 
    );
    */

    



    servStartDrawing_ = this->create_service<std_srvs::srv::Trigger>("/start_drawing", 
                                          std::bind(&PicassoArm::serviceStartDrawing, 
                                          this, std::placeholders::_1, std::placeholders::_2));

    servStopDrawing_ = this->create_service<std_srvs::srv::Trigger>("/stop_drawing", 
                                          std::bind(&PicassoArm::serviceStopDrawing, 
                                          this, std::placeholders::_1, std::placeholders::_2));

    servHomePose_ = this->create_service<std_srvs::srv::Trigger>("/move_to_home", 
                                          std::bind(&PicassoArm::serviceMoveToHome, 
                                          this, std::placeholders::_1, std::placeholders::_2));

    servEStop = this->create_service<std_srvs::srv::Trigger>("/e_stop", 
                                    std::bind(&PicassoArm::serviceEStop, 
                                    this, std::placeholders::_1, std::placeholders::_2));

    servNextContour_ = this->create_client<picasso_bot::srv::GetPoseArray>("/next_contour");
};

// void PicassoArm::moveToNextPose() {

//     if (current_target_index_ >= target_points_.size()) {
//         RCLCPP_INFO(this->get_logger(), "All target points reached.");
//         timer_->cancel();
//         return;
//     }
//     auto node_shared = shared_from_this();
//     auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");
   
//     move_group_interface.setEndEffectorLink("tool0");

//     geometry_msgs::msg::Pose target_pose;
//     target_pose.position = target_points_[current_target_index_];

//     move_group_interface.setPoseTarget(target_pose);
   
//     auto const [success, plan] = [&move_group_interface]{
//         moveit::planning_interface::MoveGroupInterface::Plan msg;
//         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//         return std::make_pair(ok, msg);
//     }();

//     if(success) {
//         move_group_interface.execute(plan);
//         RCLCPP_INFO(this->get_logger(), "Moved to target point %zu", current_target_index_);
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Planning failed for target point %zu!", current_target_index_);
//     }
// }


void PicassoArm::moveToPose() {

    auto node_shared = shared_from_this();
    auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");

    move_group_interface.setEndEffectorLink("tool0");

    // Set a target Pose
   auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.5;
    msg.position.y = 0.5;
    msg.position.z = 0.5;
    return msg;
   }();
   
    move_group_interface.setPoseTarget(target_pose);
 
    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();


    // Execute the plan
    if(success) {
     move_group_interface.execute(plan);
     RCLCPP_WARN(this->get_logger(), "Completed function");               
     timer_->cancel();

     
    } else {
     RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
}
// void PicassoArm::toolpath_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), "Received toolpath with %zu poses.", msg->poses.size());

//     target_points_.clear();
//     current_target_index_ = 0;

//     for (const auto& pose : msg->poses) {
//         target_points_.push_back(pose.position);
//     }

//     if (!target_points_.empty()) {
//         RCLCPP_INFO(this->get_logger(), "Toolpath loaded. Starting movement...");
//         timer_->reset();  // Restart timer to begin moving
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Received empty toolpath.");
//     }
// }

/*
How to use below service:
// Service already set up.

geometry_msgs::msg::PoseArray contourMsg = getNextContour();   // Request next contour

if (contourMsg.poses.size() == 0) {
    // No contour to receive and/or end of contours.
}

// contourMsg will contain a header and a vector of geometry_msgs::msg::Pose

// to access poses:
geometry_msgs::msg::Pose goalPose = contourMsg.poses.at(X); // where X is the index of the goal pose

// if you wanted a function that itterates though the list:
while (contourMsg.poses.size() > index) {
    geometry_msgs::msg::Pose goalPose = contourMsg.poses.at(index);
    index++;
}

*/

void PicassoArm::serviceStartDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Drawing started.");
    // TO DO: Start drawing loop
}

void PicassoArm::serviceStopDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Drawing stopped.");
    // TO DO: Stop drawing loop
}

void PicassoArm::serviceMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "moving to home");
    // TO DO: Move to home pose
}

void PicassoArm::serviceEStop(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    RCLCPP_WARN(this->get_logger(), "E-stop activated.");
    // TO DO: Set E-Stop state to true
}

void PicassoArm::serviceNextContourRequest(void) {
    toolPathMsg_ = geometry_msgs::msg::PoseArray();
    toolPathIndex_ = 0;

    serviceWait<picasso_bot::srv::GetPoseArray>(servNextContour_);

    auto request = std::make_shared<picasso_bot::srv::GetPoseArray::Request>();
    auto weak_this = std::weak_ptr<PicassoArm>(
        std::static_pointer_cast<PicassoArm>(this->shared_from_this()));

    servNextContour_->async_send_request(request,
        [weak_this, this](rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future) {
            
        auto shared_this = weak_this.lock();
        if (shared_this) {
            shared_this->serviceNextContourRespose(future);

        } else {
            std::string serviceName = std::string(servNextContour_->get_service_name());
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot process service '%s'.", serviceName);
        }
    });

    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", std::string(servNextContour_->get_service_name()));
}


void PicassoArm::serviceNextContourRespose(rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future) {
    auto result = future.get();
    std::string serviceName = std::string(servNextContour_->get_service_name());

    if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Service '%s' called successfully.", serviceName);

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed.", serviceName);
    }
}

geometry_msgs::msg::PoseArray PicassoArm::getNextContour(void) {
    serviceNextContourRequest();
    std::chrono::milliseconds sleepTime = std::chrono::milliseconds(100);
    unsigned int maxAttempts = 10;
    unsigned int attempts = 0;

    while (toolPathMsg_.poses.size() == 0 && attempts < maxAttempts) {
        std::this_thread::sleep_for(sleepTime);
        attempts++;
    }

    if (toolPathMsg_.poses.size() == 0) {
        if (prevContourExists_ == false) {
            RCLCPP_WARN(this->get_logger(), "No contour received after %u ms.", maxAttempts * sleepTime.count());
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Drawing complete.");
        }
        
        return toolPathMsg_;
    }

    prevContourExists_ = true;
    RCLCPP_INFO(this->get_logger(), "Contour received.");

    return toolPathMsg_;
}