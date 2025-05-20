#include "picasso_arm.h"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <thread>


using moveit::planning_interface::MoveGroupInterface;

PicassoArm::PicassoArm(void) : Node("picasso_arm") {
    RCLCPP_INFO(this->get_logger(), "PicassoArm node initialized.");

    // timer_ = this->create_wall_timer(
    //     std::chrono::seconds(5), 
    //     std::bind(&PicassoArm::planCartesianPath, this) 
    // );

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


void PicassoArm::toolpath_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received toolpath with %zu poses.", msg->poses.size());

    target_points_.clear();
    current_target_index_ = 0;

    for (const auto& pose : msg->poses) {
        target_points_.push_back(pose.position);
    }

    if (!target_points_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Toolpath loaded. Starting movement...");
        timer_->reset();  // Restart timer to begin moving
    } else {
        RCLCPP_WARN(this->get_logger(), "Received empty toolpath.");
    }
}

std::vector<geometry_msgs::msg::Pose> PicassoArm::fetchWaypoints() {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Get current pose as the starting waypoint
    auto move_group = MoveGroupInterface(this->shared_from_this(), "ur_manipulator");
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(current_pose);
    RCLCPP_INFO(this->get_logger(), "Current pose added as starting waypoint.");

    // Get the latest contour
    auto pathMsg = getNextContour();
    if (pathMsg.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No contour received. Returning current pose only.");
        return waypoints;
    }

    // Append all poses from contour
    waypoints.insert(waypoints.end(), pathMsg.poses.begin(), pathMsg.poses.end());
    RCLCPP_INFO(this->get_logger(), "%zu waypoints fetched.", waypoints.size());

    return waypoints;
}

bool PicassoArm::planCartesianPath(const std::vector<geometry_msgs::msg::Pose> &waypoints, moveit_msgs::msg::RobotTrajectory &trajectory) {
    auto move_group = MoveGroupInterface(this->shared_from_this(), "ur_manipulator");
    move_group.setEndEffectorLink("wrist_link_3");

    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction > 0.95) {
        RCLCPP_INFO(this->get_logger(), "Cartesian path planned successfully (%.2f%%).", fraction * 100.0);
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "Cartesian path planning incomplete (%.2f%%).", fraction * 100.0);
        return false;
    }
}

bool PicassoArm::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    auto move_group = MoveGroupInterface(this->shared_from_this(), "ur_manipulator");
    move_group.setEndEffectorLink("wrist_link_3");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    RCLCPP_INFO(this->get_logger(), "Executing trajectory...");

    move_group.execute(plan);
    return true;
}


void PicassoArm::serviceStartDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    (void)request;  // Unused parameter

    auto waypoints = fetchWaypoints();
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    if (planCartesianPath(waypoints, trajectory)) {
        if (executeTrajectory(trajectory)) {
            response->success = true;
            response->message = "Drawing started and path executed.";
        } else {
            response->success = false;
            response->message = "Trajectory execution failed.";
        }
    } else {
        response->success = false;
        response->message = "Cartesian path planning failed.";
    }
}

void PicassoArm::serviceStopDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    (void)request;  // Unused parameter
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Drawing stop request received.");

    if (timer_ && timer_->is_ready()) {
        timer_->cancel();   // ← stops any active timer
    }

    response->success = true;
    response->message = "Stopped drawing.";
    // TO DO: Stop drawing loop
}

void PicassoArm::serviceMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    (void)request;  // Unused parameter
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "moving to home");

    auto move_group = moveit::planning_interface::MoveGroupInterface(this->shared_from_this(), "ur_manipulator");
    move_group.setNamedTarget("up");

    bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);
    response->success = success;

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Robot moved to home position successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to move robot to home position.");
    }
}

void PicassoArm::serviceEStop(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    (void)request;  // Unused parameter
    response->success = true;
    RCLCPP_WARN(this->get_logger(), "E-STOP request received.");

    auto move_group = MoveGroupInterface(shared_from_this(), "ur_manipulator");
    move_group.stop();    // ← emergency stop

    response->success = true;
    response->message = "Emergency Stop activated.";
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
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot process service '%s'.", serviceName.c_str());
        }
    });

    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", std::string(servNextContour_->get_service_name()).c_str());
}


void PicassoArm::serviceNextContourRespose(rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future) {
    auto result = future.get();
    std::string serviceName = std::string(servNextContour_->get_service_name());

    if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Service '%s' called successfully.", serviceName.c_str());

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed.", serviceName.c_str());
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
            RCLCPP_WARN(this->get_logger(), "No contour received after %ld ms.",maxAttempts * sleepTime.count());
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Drawing complete.");
        }
        
        return toolPathMsg_;
    }

    prevContourExists_ = true;
    RCLCPP_INFO(this->get_logger(), "Contour received.");

    return toolPathMsg_;
}

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


// void PicassoArm::moveToPose() {

//     auto node_shared = shared_from_this();
//     auto move_group_interface = MoveGroupInterface(node_shared, "ur_manipulator");

//     move_group_interface.setEndEffectorLink("tool0");

//     // Set a target Pose
//    auto const target_pose = []{
//     geometry_msgs::msg::Pose msg;
//     msg.position.x = 0.5;
//     msg.position.y = 0.5;
//     msg.position.z = 0.5;
//     return msg;
//    }();
   
//     move_group_interface.setPoseTarget(target_pose);
 
//     // Create a plan to that target pose
//     auto const [success, plan] = [&move_group_interface]{
//         moveit::planning_interface::MoveGroupInterface::Plan msg;
//         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//         return std::make_pair(ok, msg);
//     }();


//     // Execute the plan
//     if(success) {
//      move_group_interface.execute(plan);
//      RCLCPP_WARN(this->get_logger(), "Completed function");               
//      timer_->cancel();

     
//     } else {
//      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
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

// get path function (void) - reutun vec<pose>
// calculate traj function (vec<pose>) - return traj
// move arm function (traj) - return bool
// void PicassoArm::cartesianPath(){

    

//     using moveit::planning_interface::MoveGroupInterface;
//     auto move_group_interface = MoveGroupInterface(this->shared_from_this(), "ur_manipulator");

//     auto const end_effector_pose = move_group_interface.getCurrentPose();

//     RCLCPP_INFO(this->get_logger(), "Starting Cartesian path movement.");

//     // Get the latest contour (calls your existing service + fills toolPathMsg_)
//     auto pathMsg = getNextContour();

//     if (pathMsg.poses.empty()) {
//         RCLCPP_WARN(this->get_logger(), "No waypoints received. Aborting Cartesian move.");
//         return;
//     }

//     auto move_group = MoveGroupInterface(this->shared_from_this(), "ur_manipulator");
//     move_group.setEndEffectorLink("wrist_link_3");

//     // Convert PoseArray to vector of waypoints
//     ///std::vector<geometry_msgs::msg::Pose> waypoints(path.poses);
    

//     moveit_msgs::msg::RobotTrajectory trajectory;
//     double fraction = move_group.computeCartesianPath(pathMsg.poses, 0.01, 0.0, trajectory);  // 1cm step, no jump threshold

//     if (fraction > 0.95) {
//         RCLCPP_INFO(this->get_logger(), "Cartesian path planned successfully (%.2f%%). Executing...", fraction * 100.0);
//         move_group.execute(trajectory);
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Cartesian path planning incomplete (%.2f%%). Not executing.", fraction * 100.0);
//     }
// }