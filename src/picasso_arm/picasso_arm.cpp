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

    kinematicModel_ = groupInterface_->getRobotModel();
    if (!kinematicModel_) {
        RCLCPP_ERROR(owningNode_->get_logger(), "Failed to get robot model from MoveGroupInterface.");
        return;
    }

    if (!kinematicModel_->getJointModelGroup(moveGroupName_)) {
        RCLCPP_ERROR(owningNode_->get_logger(), "Failed to get joint model group '%s'. Check your SRDF.", moveGroupName_.c_str());
        return;
    }

    RCLCPP_INFO(owningNode_->get_logger(), "Move Controller initialized.");
}

bool MoveControl::poseToRobotState(const geometry_msgs::msg::Pose &pose, 
                                    moveit::core::RobotState &robotState) {
    robotState.setToDefaultValues();
    Eigen::Isometry3d poseEigen;
    tf2::fromMsg(pose,poseEigen);
    bool success = robotState.setFromIK(robotState.getJointModelGroup(moveGroupName_), 
                                        pose, 
                                        1.0);

    if (success) {
        RCLCPP_INFO(owningNode_->get_logger(), "Converted pose successfully!");
        return true;

    } else {
        RCLCPP_WARN(owningNode_->get_logger(), "Failed to convert pose!");
        return false;
    }
}

bool MoveControl::planTrajectoryPoint(geometry_msgs::msg::Pose &startPose, 
                                    geometry_msgs::msg::Pose &goalPose, 
                                    moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // Abort if kinematic model is not initialized
    if (kinematicModel_ == NULL) {
        RCLCPP_WARN(owningNode_->get_logger(), "Aborting planning, kinematic model is not initialized!");    
        return false;
    }

    // Abort if interface is not initialized
    if (startPose == goalPose) {
        RCLCPP_WARN(owningNode_->get_logger(), "Aborting planning, start and goal poses are the same!");    
        return false;
    }
    
    groupInterface_->setPoseTarget(goalPose);
    moveit::core::RobotState startState(kinematicModel_);
    poseToRobotState(startPose, startState);
    groupInterface_->setStartState(startState);
 
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

bool MoveControl::planTrajectoryPath(geometry_msgs::msg::Pose &startPose,
                                    geometry_msgs::msg::PoseArray &waypoints, 
                                    moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // Abort if kinematic model is not initialized
    if (!kinematicModel_ == NULL) {
        RCLCPP_WARN(owningNode_->get_logger(), "Aborting planning, UR robot is not connected!");    
        return false;
    }
    
    // Abort if waypoints are empty
    if (waypoints.poses.size() <= 0) {
        RCLCPP_WARN(owningNode_->get_logger(), "Aborting planning, waypoints are empty!");    
        return false;
    }

    const double trajStep = 0.01;    // [m] trajectory goal step size
    const double maxVariance = 0.01; // [m] 1cm. Maximum allowed variance from the path
    const double completnessMin = 50.0; // [%] 50% of the path must be completed for success

    // Add start pose to waypoints
    std::vector<geometry_msgs::msg::Pose> tempVec;
    tempVec.reserve(waypoints.poses.size() + 1);
    tempVec.push_back(startPose);
    tempVec.insert(tempVec.end(), waypoints.poses.begin(), waypoints.poses.end());
    waypoints.poses = tempVec;

    // Get starting state.
    moveit::core::RobotState startState(kinematicModel_);
    poseToRobotState(startPose, startState);
    groupInterface_->setStartState(startState);

    const double completness = groupInterface_->computeCartesianPath(waypoints.poses, 
                                                                    trajStep, 
                                                                    maxVariance, 
                                                                    plan.trajectory_);

    if (completness >= completnessMin && plan.trajectory_.joint_trajectory.joint_names.size() > 0) {
        RCLCPP_INFO(owningNode_->get_logger(), "Planning successful!");
        return true;

    } else {
        std::string errorMsg;

        if (completness < completnessMin) {
            errorMsg = "completeness < %f%", completnessMin;

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

    servTotalLines_ = this->create_client<picasso_bot::srv::GetTotalLines>("/total_lines");

    RCLCPP_INFO(this->get_logger(), "PicassoArm node initialized.");
}

void PicassoArm::drawImage(void) {
    std::chrono::milliseconds sleepTime = std::chrono::milliseconds(100);
    unsigned int maxAttempts = 50;  // 5 secconds
    unsigned int attempts = 0;
    bool drawingComplete = false;
    
    while (!drawingComplete) {
        bool drawingComplete = getNextContour();

        RCLCPP_INFO(this->get_logger(), "Drawing line [%u/%u]", currentLine_, totalLines_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        geometry_msgs::msg::Pose startPose = moveController_->groupInterface_->getCurrentPose().pose;
        bool success = moveController_->planTrajectoryPath(startPose, toolPathMsg_, plan);

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

    // TODO: Move to home pose
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



void PicassoArm::getCurrentPose() {
    
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
    // TO DO: LAUNCH AS OWN THREAD
    drawImage();
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
    moveController_->groupInterface_->stop(); // Stop the robot
    RCLCPP_WARN(this->get_logger(), "E-stop activated.");
}



void PicassoArm::serviceTotalLinesRequest(void) {
    totalLines_ = 0;
    totalLinesResponce_ = false;
    serviceLogName_ = std::string(servTotalLines_->get_service_name());

    serviceWait<picasso_bot::srv::GetTotalLines>(servTotalLines_);

    auto request = std::make_shared<picasso_bot::srv::GetTotalLines::Request>();
    auto weak_this = std::weak_ptr<PicassoArm>(
        std::static_pointer_cast<PicassoArm>(this->shared_from_this()));

    servTotalLines_->async_send_request(request,
        [weak_this, this](rclcpp::Client<picasso_bot::srv::GetTotalLines>::SharedFuture future) {
            
        auto shared_this = weak_this.lock();
        if (shared_this) {
            shared_this->serviceTotalLinesRespose(future);

        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot process service '%s'.", serviceLogName_.c_str());
        }
    });

    
    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", serviceLogName_.c_str());
}

void PicassoArm::serviceTotalLinesRespose(rclcpp::Client<picasso_bot::srv::GetTotalLines>::SharedFuture future) {
    auto result = future.get();
    serviceLogName_ = std::string(servNextContour_->get_service_name());
    
    if (result->success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        totalLines_ = result->amount;
        RCLCPP_INFO(this->get_logger(), "Service '%s' called successfully.", serviceLogName_.c_str());

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed.", serviceLogName_.c_str());
    }

    totalLinesResponce_ = true;
}

void PicassoArm::serviceNextContourRequest(void) {
    serviceLogName_ = std::string(servTotalLines_->get_service_name());
    toolPathMsg_ = geometry_msgs::msg::PoseArray();
    contourResponce_ = false;

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
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "MainWindow object expired, cannot process service '%s'.", serviceLogName_.c_str());
            return;
        }
    });

    RCLCPP_INFO(this->get_logger(), "Service '%s' request sent.", serviceLogName_.c_str());
}

void PicassoArm::serviceNextContourRespose(rclcpp::Client<picasso_bot::srv::GetPoseArray>::SharedFuture future) {
    auto result = future.get();
    serviceLogName_ = std::string(servNextContour_->get_service_name());
    contourResponce_ = true;

    if (result->success) {
        currentLine_ = result->line_id;
        totalLines_ = result->total_lines;
        toolPathMsg_ = result->poses;
        RCLCPP_INFO(this->get_logger(), "Service '%s' called successfully.", serviceLogName_.c_str());

    } else {
        RCLCPP_WARN(this->get_logger(), "Service '%s' failed.", serviceLogName_.c_str());
    }
}

bool PicassoArm::getNextContour(void) {
    serviceNextContourRequest();
    std::chrono::milliseconds sleepTime = std::chrono::milliseconds(100);
    unsigned int maxAttempts = 10;
    unsigned int attempts = 0;

    while (!contourResponce_ && attempts < maxAttempts) {
        std::this_thread::sleep_for(sleepTime);
        attempts++;
    }

    if (toolPathMsg_.poses.size() <= 0) {

        if (!prevContourExists_) {
            RCLCPP_WARN(this->get_logger(), "No contour received after %u ms.", maxAttempts * sleepTime.count());
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Drawing complete.");
        }
        
        return false;
    }

    prevContourExists_ = true;
    RCLCPP_INFO(this->get_logger(), "Contour received.");

    return true;
}