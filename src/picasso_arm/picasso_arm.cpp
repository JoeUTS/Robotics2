#include "picasso_arm.h"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

MoveControl::MoveControl(const std::shared_ptr<rclcpp::Node> owningNode, 
                         const std::string moveGroupName, 
                         const std::string endEffectorLink) : 
                                owningNode_(owningNode), 
                                moveGroupName_(moveGroupName), 
                                endEffectorLink_(endEffectorLink) {

    groupInterface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(owningNode_, moveGroupName_);
    groupInterface_->setEndEffectorLink(endEffectorLink_);
    groupInterface_->startStateMonitor();
    groupInterface_->setNumPlanningAttempts(5);
    groupInterface_->setPlanningTime(5.0);                          // [s] 5 sec
    groupInterface_->setGoalPositionTolerance(5e-5);                // [m] 5mm
    groupInterface_->setGoalOrientationTolerance(5 * M_PI / 180);   // [rad] 5 deg
    groupInterface_->setGoalJointTolerance(5 * M_PI / 180);         // [rad] 5 deg

    RCLCPP_INFO(owningNode_->get_logger(), "Move Controller initialized.");
}

void MoveControl::moveToHome(void) {
    groupInterface_->setNamedTarget("up");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(groupInterface_->plan(plan));

    if (success) {
        RCLCPP_INFO(owningNode_->get_logger(), "Planned to 'home' position. Executing...");
        groupInterface_->execute(plan);
        
    } else {
        RCLCPP_WARN(owningNode_->get_logger(), "Failed to plan to 'home' position.");
    }
}

bool MoveControl::planTrajectoryPoint(geometry_msgs::msg::Pose &goalPose, 
                                    moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // Create a plan
    auto const [success, calulatedPlan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(groupInterface_->plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        plan = calulatedPlan;
        groupInterface_->setStartStateToCurrentState();
        RCLCPP_INFO(owningNode_->get_logger(), "Planning successful!");

        return true;
     
    } else {
        RCLCPP_ERROR(owningNode_->get_logger(), "Planning failed!");

        return false;
    }
}

bool MoveControl::planTrajectoryPath(geometry_msgs::msg::PoseArray &waypoints, 
                                    moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    const double trajStep = 0.01;       // [m] trajectory goal step size
    const double maxVariance = 0.01;    // [m] 1cm. Maximum allowed variance from the path
    const double completnessMin = 50.0; // [%] 50% of the path must be completed for success

    std::vector<geometry_msgs::msg::Pose> tempVec = waypoints.poses;
    const double completness = groupInterface_->computeCartesianPath(tempVec, 
                                                                    trajStep, 
                                                                    maxVariance, 
                                                                    plan.trajectory_);

    if (completness >= completnessMin && plan.trajectory_.joint_trajectory.joint_names.size() > 0) {
        RCLCPP_INFO(owningNode_->get_logger(), "Planning successful!");
        return true;

    } else {
        std::string errorMsg;

        if (completness < completnessMin) {
            errorMsg = "poor trajectory completion, " + std::to_string(completness) + "%";

        } else {
            errorMsg = "empty plan.";
        }

        RCLCPP_ERROR(owningNode_->get_logger(), "Planning failed, %s!", errorMsg.c_str());
        return false;
    }
}


bool MoveControl::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    // Abort if plan is empty
    if (plan.trajectory_.joint_trajectory.joint_names.size() <= 0) {
        RCLCPP_WARN(owningNode_->get_logger(), "Aborting movement, empty plan!");    
        return false;
    }

    // Execute plan
    groupInterface_->execute(plan);
    RCLCPP_WARN(owningNode_->get_logger(), "Executing planned trajectory!");               

    return true;
}




PicassoArm::PicassoArm(void) : Node("picasso_arm"), 
                                moveGroupName_("ur_manipulator"), 
                                endEffectorLink_("tool0") {
    // Setting flags
    prevContourExists_ = false;

    // Services
    servConnectUR_ = this->create_service<std_srvs::srv::Trigger>("/connect_ur",
                                          std::bind(&PicassoArm::serviceConnectUR, 
                                          this, std::placeholders::_1, std::placeholders::_2));

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

    //servTotalLines_ = this->create_client<picasso_bot::srv::GetTotalLines>("/total_lines");

    RCLCPP_INFO(this->get_logger(), "PicassoArm node initialised.");
}

void PicassoArm::resumeDrawing() {
    // I dont know what this is supposed to do but whatever it is, it wont work.
    // I have written out your logic to show why this wont work.
    if (!drawing_active_) {         // Is the drawing active flag false?
        drawing_active_ = true;     // Set drawing flag to true
        drawing_paused_ = false;    // set pause flag to false
        std::thread(&PicassoArm::drawImage, this).detach(); // launch new thread to start new drawing loop
    } else {
        drawing_paused_ = false;    // if drawing is active set pause flag to false
    }

    // I presume that you want to be able to pause and continue drawing.
    // to do so, your code needs to be inside of the drawing loop.
    // Personally I would do this at the start of the loop:
    //        while (drawing_paused_) {
    //            std::this_thread::sleep_for(std::chrono::milliseconds(250));
    //        }
    // Then you can have the pause and resume functions set the drawing_paused_ flag for it to work.
}

void PicassoArm::pauseDrawing() {
    drawing_paused_ = true;
}


void PicassoArm::drawImage(void) {
    std::chrono::milliseconds sleepTime = std::chrono::milliseconds(100);
    unsigned int maxAttempts = 50;  // 5 secconds
    unsigned int attempts = 0;
    bool receivedContour = true;

    //Georgios - Made this loop for robot to pause drawing and continue from where it stopped.
    // not sure if this is the best way to do it.

    // while (drawing_active_ && currentLine_ < totalLines_) {
    // if (drawing_paused_) {
    //     RCLCPP_INFO(this->get_logger(), "Drawing paused. Waiting to resume...");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(250));
    //     continue;
    // }

    // geometry_msgs::msg::PoseArray currentLine = toolPathContours_[currentLine_];
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // geometry_msgs::msg::Pose startPose = moveController_->groupInterface_->getCurrentPose().pose;
    // bool success = moveController_->planTrajectoryPath(startPose, currentLine, plan);
    //     if (!success) {
    //         RCLCPP_ERROR(this->get_logger(), "Planning failed on line %u. Aborting.", currentLine_);
    //         break;
    //     }

    //     success = moveController_->executePlan(plan);
    //     if (!success) {
    //         RCLCPP_ERROR(this->get_logger(), "Execution failed on line %u. Aborting.", currentLine_);
    //         break;
    //     }
    //     currentLine_++; // Increment the current line after successful execution
    // }
    // drawing_active_ = false; // Stop drawing after the last line is drawn
    // RCLCPP_INFO(this->get_logger(), "Drawing completed. Moving to HOME!");
    // moveToHome();

    while (receivedContour) {
        while (drawing_paused_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

        receivedContour = getNextContour();

        if (!receivedContour) { // Exit loop if no contour is received
            break;
        }

        RCLCPP_INFO(this->get_logger(), "Drawing line [%u/%u]", currentLine_, totalLines_);
        for (unsigned int i = 0; i < toolPathMsg_.poses.size(); i++) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = moveController_->planTrajectoryPoint(toolPathMsg_.poses.at(i), plan);

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Drawing aborted! Failed to plan trajectory");
                break;
            }

            success = moveController_->executePlan(plan);

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Drawing aborted! Failed to execute");
                break;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Line [%u/%u] drawn successfully.", currentLine_, totalLines_);
    }

    RCLCPP_INFO(this->get_logger(), "Drawing completed. Moving to HOME!");
    moveController_->moveToHome();
}

void PicassoArm::serviceConnectUR(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    // TO DO: are there any other things we need to connect to?
    moveController_ = std::make_shared<MoveControl>(this->shared_from_this(), 
                                                    moveGroupName_,
                                                    endEffectorLink_);
    
    RCLCPP_INFO(this->get_logger(), "Connected to UR.");
}

void PicassoArm::serviceStartDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Drawing started.");
    std::thread(&PicassoArm::drawImage, this).detach();
}

void PicassoArm::serviceStopDrawing(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    
    response->success = true;
    pauseDrawing(); // Pause the drawing process
    
    RCLCPP_INFO(this->get_logger(), "Drawing stopped.");
    // TO DO: Stop drawing loop
}

void PicassoArm::serviceMoveToHome(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "moving to home");
    moveController_->moveToHome();
    RCLCPP_INFO(this->get_logger(), "Moved to home.");
}

void PicassoArm::serviceEStop(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    moveController_->groupInterface_->stop(); // Stop the robot
    RCLCPP_WARN(this->get_logger(), "E-stop activated.");
}

void PicassoArm::serviceNextContourRequest(void) {
    serviceLogName_ = std::string(servNextContour_->get_service_name());
    toolPathMsg_ = geometry_msgs::msg::PoseArray();
    contourResponce_ = false;

    serviceWait<picasso_bot::srv::GetPoseArray>(servNextContour_);

    auto request = std::make_shared<picasso_bot::srv::GetPoseArray::Request>();
    auto result = servNextContour_->async_send_request(request);
    
    std::future_status status = result.wait_for(std::chrono::seconds(5)); 
    if (status == std::future_status::ready) {
        std::shared_ptr<picasso_bot::srv::GetPoseArray::Response> response = result.get();
        RCLCPP_INFO(this->get_logger(), "Contour received");

        currentLine_ = response->line_id;
        totalLines_ = response->total_lines;
        toolPathMsg_ = response->poses;

        RCLCPP_INFO(this->get_logger(), "Contour size: %zu", toolPathMsg_.poses.size());
        RCLCPP_INFO(this->get_logger(), "total lines: %u", totalLines_);
        RCLCPP_INFO(this->get_logger(), "current line: %u", currentLine_);

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed.", serviceLogName_.c_str());
        currentLine_ = 0;
        totalLines_ = 0;
        toolPathMsg_ = geometry_msgs::msg::PoseArray();
    }

    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", serviceLogName_.c_str());
}

bool PicassoArm::getNextContour(void) {
    serviceNextContourRequest();
    RCLCPP_ERROR(this->get_logger(), "Contour size: %u", toolPathMsg_.poses.size());
    RCLCPP_ERROR(this->get_logger(), "Total lines: %u", totalLines_);
    RCLCPP_ERROR(this->get_logger(), "Current line: %u", currentLine_);

    if (toolPathMsg_.poses.size() <= 0) {

        if (!prevContourExists_) {
            RCLCPP_ERROR(this->get_logger(), "No contour received");
        } else {
            RCLCPP_INFO(this->get_logger(), "Drawing complete.");
        }
        
        return false;
    }

    prevContourExists_ = true;
    RCLCPP_INFO(this->get_logger(), "Contour received.");

    return true;
}