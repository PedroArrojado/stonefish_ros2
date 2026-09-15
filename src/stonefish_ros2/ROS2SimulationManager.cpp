/*    
    This file is a part of stonefish_ros2.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROS2SimulationManager.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2025 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2ScenarioParser.h"
#include "stonefish_ros2/ROS2Interface.h"

#include "stonefish_ros2/msg/thruster_state.hpp"
#include "stonefish_ros2/msg/debug_physics.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/TurbulenceMixer.h>
#include <Stonefish/entities/forcefields/GustMixer.h>
#include <Stonefish/entities/forcefields/VelocityField.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/joints/FixedJoint.h>

#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/scalar/Altimeter.h>
#include <Stonefish/sensors/scalar/Barometer.h>
#include <Stonefish/sensors/scalar/RangeFinder.h>
#include <Stonefish/sensors/scalar/Anemometer.h>

#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/Lidar.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>

#include <Stonefish/comms/USBL.h>
#include <Stonefish/comms/OpticalModem.h>

#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/RotatingElement.h>
#include <Stonefish/actuators/ControlSurface.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/Light.h>

#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/Entity.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/StaticEntity.h>
#include <Stonefish/entities/FeatherstoneEntity.h>

using namespace std::placeholders;

namespace sf
{

ROS2SimulationManager::ROS2SimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath, std::string dataPath, const std::shared_ptr<rclcpp::Node>& nh)
	: SimulationManager(stepsPerSecond, Solver::SI, CollisionFilter::EXCLUSIVE), 
        accumulatedSimUs_(0), scenarioPath_(scenarioFilePath), dataPath_(dataPath), nh_(nh)
{
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    interface_ = std::make_shared<ROS2Interface>(nh_);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
    
    // NEW: Mechanisms for clock publishing (use_sim_time: sync nodes with sim time)
    clockPub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));

    // NEW: Authority node mechanisms
    authorityVote_ = nh->create_subscription<stonefish_ros2::msg::AuthorityNodeVote>(
        "authority_node_vote", 
        rclcpp::QoS(1), 
        std::bind(&ROS2SimulationManager::AuthorityNodeStepVote, this, std::placeholders::_1));
    
    authorityStep_ = nh->create_subscription<stonefish_ros2::msg::AuthorityNodeStep>(
        "authority_node_step",
        rclcpp::QoS(10),
        std::bind(&ROS2SimulationManager::AuthorityNodeStepGrant, this, std::placeholders::_1));
    
    // NEW: Wave height publisher for monitoring wave height at several locations (for spectral params)
    waveHeights_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("/stonefish/wave_heights", rclcpp::QoS(1));

    nominal_dt_us_ = static_cast<uint64_t>(std::llround(1e6 / static_cast<double>(stepsPerSecond)));
    if(nominal_dt_us_ == 0) nominal_dt_us_ = 1;
    customRTF = 1.0;
}

ROS2SimulationManager::~ROS2SimulationManager()
{
}

// NEW: Authority node Sign in function
int64_t ROS2SimulationManager::AuthorityNodeSignIn(
    stonefish_ros2::srv::AuthorityNodeSignIn::Request::SharedPtr req,
    stonefish_ros2::srv::AuthorityNodeSignIn::Response::SharedPtr res)
{
    std::string node_name = req->node_name;
    int64_t timeout_ms = (req->timeout_ms < 0) ? 0 : req->timeout_ms;

    int64_t node_id = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    {
        std::lock_guard<std::mutex> lk(authority_mutex);
        while (authorityNodes_.find(node_id) != authorityNodes_.end()) {
            node_id += 1;
        }

        AuthorityNode authorityNode;
        authorityNode.node_name            = node_name;
        authorityNode.authorityID          = node_id;
        authorityNode.authority_timeout_ms = timeout_ms;
        authorityNode.mode                 = (req->mode == 1) ? AuthorityMode::STEPPER
                                                             : AuthorityMode::VOTER;

        // VOTER default: allowed to step until it votes to hold.
        authorityNode.permissionToStep_ = true;

        // STEPPER default: active and holding immediately (zero budget) until the first grant.
        authorityNode.remaining_us_   = 0;
        authorityNode.stepper_active_ = (authorityNode.mode == AuthorityMode::STEPPER);
        authorityNode.hold_started_   = std::chrono::steady_clock::now();

        authorityNodes_.insert(std::make_pair(node_id, authorityNode));
    }
    authority_cv.notify_one();   // empty map state may have changed

    res->id = node_id;
    res->success = true;
    RCLCPP_INFO_STREAM(nh_->get_logger(),
        "Authority node '" << node_name << "' signed in with timeout "
        << timeout_ms << "ms. Got ID " << node_id);
    return node_id;
}

// NEW: Authority node sign out function
bool ROS2SimulationManager::AuthorityNodeSignOut(
    stonefish_ros2::srv::AuthorityNodeSignOut::Request::SharedPtr req,
    stonefish_ros2::srv::AuthorityNodeSignOut::Response::SharedPtr res)
{
    {
        std::lock_guard<std::mutex> lk(authority_mutex);
        auto it = authorityNodes_.find(req->id);
        if (it == authorityNodes_.end()) {
            res->success = false;
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                "Authority node with ID " << req->id << " not found for sign out.");
            return false;
        }
        RCLCPP_INFO_STREAM(nh_->get_logger(),
            "Authority node '" << it->second.node_name << "' (ID " << req->id << ") signed out.");
        authorityNodes_.erase(it);
    }
    authority_cv.notify_one();   // critical: a holding node leaving unblocks the wait
    res->success = true;
    return true;
}

// Authority node step vote subscriber callback
void ROS2SimulationManager::AuthorityNodeStepVote(const stonefish_ros2::msg::AuthorityNodeVote::SharedPtr vote)
{
    {
        std::lock_guard<std::mutex> lk(authority_mutex);
        auto it = authorityNodes_.find(vote->id);
        if (it == authorityNodes_.end()) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                "Authority node with ID " << vote->id << " not found for voting.");
            return;
        }
        if (it->second.mode != AuthorityMode::VOTER) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                "Authority node '" << it->second.node_name << "' is a STEPPER; ignoring vote.");
            return;
        }
        // Stamp the time when transitioning into a hold
        if (it->second.permissionToStep_ && !vote->vote) {
            it->second.hold_started_ = std::chrono::steady_clock::now();
        }
        it->second.permissionToStep_ = vote->vote;
    }
    authority_cv.notify_one();
}

// Authority node step grant subscriber callback
void ROS2SimulationManager::AuthorityNodeStepGrant(const stonefish_ros2::msg::AuthorityNodeStep::SharedPtr grant)
{
    {
        std::lock_guard<std::mutex> lk(authority_mutex);
        auto it = authorityNodes_.find(grant->id);
        if (it == authorityNodes_.end()) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                "Authority node with ID " << grant->id << " not found for step grant.");
            return;
        }
        AuthorityNode & node = it->second;
        if (node.mode != AuthorityMode::STEPPER) {
            RCLCPP_WARN_STREAM(nh_->get_logger(),
                "Authority node '" << node.node_name << "' is not a STEPPER; ignoring step grant.");
            return;
        }

        int64_t add_us = static_cast<int64_t>(grant->advance.sec) * 1000000LL
                       + static_cast<int64_t>(grant->advance.nanosec) / 1000LL;
        if (add_us < 0) add_us = 0;

        if (!node.stepper_active_) {
            // Reactivate from an inactive (timed-out) state with a fresh budget.
            node.stepper_active_ = true;
            node.remaining_us_   = static_cast<uint64_t>(add_us);
        } else {
            // Token-bucket accumulate: keeps total advance == sum of grants, preserves sub-dt credit.
            node.remaining_us_  += static_cast<uint64_t>(add_us);
        }

        // Still not enough for a full step? The node is alive, so re-arm its watchdog grace.
        if (node.remaining_us_ < nominal_dt_us_)
            node.hold_started_ = std::chrono::steady_clock::now();
    }
    authority_cv.notify_one();
}

// Apply wrench to entity (for robots, right now it applies on base_link (link[0])
void ROS2SimulationManager::ApplyWrenchService(
    const stonefish_ros2::srv::ApplyWrench::Request::SharedPtr req,
    stonefish_ros2::srv::ApplyWrench::Response::SharedPtr res)
{
    Vector3 force(req->wrench.force.x, req->wrench.force.y, req->wrench.force.z);
    Vector3 torque(req->wrench.torque.x, req->wrench.torque.y, req->wrench.torque.z);

    bool found = ApplyWrench(req->name, force, torque, 0);  // or hardcode 0 if no link_index in srv

    if (found)
    {
        res->success = true;
        res->message = "Wrench applied to " + req->name + ".";
    }
    else
    {
        res->success = false;
        res->message = "Entity or robot '" + req->name + "' not found or not dynamic.";
    }
}

// NEW: Changed clock logic for consitent time steps; dynamic RTFs; and sim_time clock publisher
uint64_t ROS2SimulationManager::getSimulationClock() const
{
    std::lock_guard<std::mutex> lock(clockMutex_);
    return accumulatedSimUs_;   // pure read
}

void ROS2SimulationManager::SimulationClockSleep(uint64_t us)
{
    // Pause mode
    if(customRTF <= 0.0)
    {
        while(customRTF <= 0.0)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        nextWallDeadline_    = std::chrono::steady_clock::now();
        deadlineInitialized_ = true;
        lastKnownRTF_        = customRTF;
    }

    // Re-anchor on first call or RTF change
    if(!deadlineInitialized_ || customRTF != lastKnownRTF_)
    {
        nextWallDeadline_    = std::chrono::steady_clock::now();
        deadlineInitialized_ = true;
        lastKnownRTF_        = customRTF;
    }

    // Advance absolute deadline by exactly us/RTF
    double wall_us = std::min(static_cast<double>(us) / customRTF, 1e9);
    nextWallDeadline_ +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::micro>(wall_us));

    // Lag guard
    if(nextWallDeadline_ < std::chrono::steady_clock::now() - std::chrono::milliseconds(200))
        nextWallDeadline_ = std::chrono::steady_clock::now();

    std::this_thread::sleep_until(nextWallDeadline_);

    // Commit sim time after sleeping
    std::lock_guard<std::mutex> lock(clockMutex_);
    accumulatedSimUs_ += us;
}

std::map<std::string, rclcpp::ServiceBase::SharedPtr>& ROS2SimulationManager::getServices()
{
    return srvs_;
}

std::map<std::string, rclcpp::PublisherBase::SharedPtr>& ROS2SimulationManager::getPublishers()
{
    return pubs_;
}
    
std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& ROS2SimulationManager::getSubscribers()
{
    return subs_;
}

std::map<std::string, image_transport::Publisher>& ROS2SimulationManager::getImagePublishers()
{
    return imgPubs_;
}

std::shared_ptr<image_transport::ImageTransport> ROS2SimulationManager::getImageTransportHandle()
{
    return it_;
}

std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& ROS2SimulationManager::getCameraMsgPrototypes()
{
    return cameraMsgPrototypes_;
}

std::map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::getDualImageCameraMsgPrototypes()
{
    return dualImageCameraMsgPrototypes_;
}

std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::getSonarMsgPrototypes()
{
    return sonarMsgPrototypes_;
}

void ROS2SimulationManager::AddROS2Robot(const std::shared_ptr<ROS2Robot>& robot)
{
    rosRobots_.push_back(robot);
}

bool ROS2SimulationManager::RespawnROS2Robot(const std::string& robotName, const Transform& origin)
{
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->robot_->getName() == robotName)
        {
            rosRobots_[i]->respawnOrigin_ = origin;
            rosRobots_[i]->respawnRequested_ = true;
            return true;
        }
    }
    return false;
}

void ROS2SimulationManager::BuildScenario()
{
    // Run parser
    ROS2ScenarioParser parser(this, nh_);
    bool success = parser.Parse(scenarioPath_);

    // Save log
    std::string logPath = rclcpp::get_logging_directory().string() + "/stonefish_ros2_parser.log";
    bool success2 = parser.SaveLog(logPath);

    if(!success)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Parsing of scenario file '%s' failed!", scenarioPath_.c_str());
        if(success2)
            RCLCPP_ERROR(nh_->get_logger(), "For more information check the parser log file '%s'.", logPath.c_str());
    }

    if(!success2)
        RCLCPP_ERROR(nh_->get_logger(), "Parser log file '%s' could not be saved!", logPath.c_str());

    // Standard services
    srvs_["enable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("enable_currents", std::bind(&ROS2SimulationManager::EnableCurrentsService, this, _1, _2));
    srvs_["disable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("disable_currents", std::bind(&ROS2SimulationManager::DisableCurrentsService, this, _1, _2));
    srvs_["respawn_robot"] = nh_->create_service<stonefish_ros2::srv::Respawn>("respawn_robot", std::bind(&ROS2SimulationManager::RespawnRobotService, this, _1, _2));
    
    // NEW: Services for dynamic environment manipulation
    // All environment changes could be useful here (i.e. jerlov water)
    // Currents are handled by topics, see ROS2ScenarioParser.cpp
    srvs_["respawn_static"] = nh_->create_service<stonefish_ros2::srv::Respawn>("respawn_static", std::bind(&ROS2SimulationManager::RespawnStaticService, this, _1, _2));
    srvs_["respawn_dynamic"] = nh_->create_service<stonefish_ros2::srv::Respawn>("respawn_dynamic", std::bind(&ROS2SimulationManager::RespawnDynamicService, this, _1, _2));
    srvs_["set_rtf"] = nh_->create_service<stonefish_ros2::srv::SetRTF>("set_rtf", std::bind(&ROS2SimulationManager::setRTFService, this, _1, _2));
    srvs_["change_ocean"] = nh_->create_service<stonefish_ros2::srv::ChangeOcean>("change_ocean", std::bind(&ROS2SimulationManager::changeOceanService, this, _1, _2));
    srvs_["spawn_dynamic"] = nh_->create_service<stonefish_ros2::srv::SpawnObject>("dynamic_spawn", std::bind(&ROS2SimulationManager::DynamicSpawnService, this, _1, _2));
    srvs_["spawn_static"] = nh_->create_service<stonefish_ros2::srv::SpawnObject>("static_spawn", std::bind(&ROS2SimulationManager::StaticSpawnService, this, _1, _2));
    srvs_["spawn_robot"] = nh_->create_service<stonefish_ros2::srv::SpawnObject>("robot_spawn", std::bind(&ROS2SimulationManager::RobotSpawnService, this, _1, _2));
    srvs_["delete_entity"] = nh_->create_service<stonefish_ros2::srv::DeleteObject>("delete_entity", std::bind(&ROS2SimulationManager::DeleteObjectService, this, _1, _2));
    srvs_["apply_wrench"] = nh_->create_service<stonefish_ros2::srv::ApplyWrench>("apply_wrench", std::bind(&ROS2SimulationManager::ApplyWrenchService, this, _1, _2));
    srvs_["change_fog"] = nh_->create_service<stonefish_ros2::srv::ChangeFog>("change_fog", std::bind(&ROS2SimulationManager::changeFogService, this, _1, _2));
    srvs_["authority_node_signin"] = nh_->create_service<stonefish_ros2::srv::AuthorityNodeSignIn>("authority_node_signin", std::bind(&ROS2SimulationManager::AuthorityNodeSignIn, this, _1, _2));
    srvs_["authority_node_signout"] = nh_->create_service<stonefish_ros2::srv::AuthorityNodeSignOut>("authority_node_signout", std::bind(&ROS2SimulationManager::AuthorityNodeSignOut, this, _1, _2));
}

void ROS2SimulationManager::DestroyScenario()
{
    for(auto it = imgPubs_.begin(); it != imgPubs_.end(); ++it)
        it->second.shutdown();

    pubs_.clear();
    imgPubs_.clear();
    subs_.clear();
    srvs_.clear();
    cameraMsgPrototypes_.clear();
    sonarMsgPrototypes_.clear();
    rosRobots_.clear();

    SimulationManager::DestroyScenario();
}

// NEW: attempt at deleting robots via schedule to avoid race conditions
// (not working always segfaults) Probably missing some instances that are left dangling 
// (fix should be on stonefish library side)
bool ROS2SimulationManager::HasPendingStructuralChanges()
{
    std::lock_guard<std::mutex> lk1(spawnQueueMutex_);
    std::lock_guard<std::mutex> lk2(deleteQueueMutex_);
    return !robotSpawnQueue_.empty() || !robotDeleteQueue_.empty();
}

void ROS2SimulationManager::ApplyStructuralChanges()
{
    DrainRobotSpawnQueue();    // ParseRobot -> BuildObject now has a live GL context
    DrainRobotDeleteQueue();   // RemoveRobot's vision-sensor GL teardown is now on the GL thread
}
	
void ROS2SimulationManager::SimulationStepCompleted(Scalar timeStep)
{
    (void)timeStep; // Suppress warning
    
    // NEW: Clock handling with sim_time
    // plus Authority nodes management

    // Publish sim clock from actual Stonefish time for speed
    uint64_t sim_us = getSimulationClock();
    rosgraph_msgs::msg::Clock clockMsg;
    clockMsg.clock.sec    = static_cast<int32_t>(sim_us / 1000000);
    clockMsg.clock.nanosec = static_cast<uint32_t>((sim_us % 1000000) * 1000);
    clockPub_->publish(clockMsg);

    currentSimTime_ = rclcpp::Time(
        static_cast<int32_t>(sim_us / 1000000),
        static_cast<uint32_t>((sim_us % 1000000) * 1000),
        RCL_ROS_TIME
    );

    const uint64_t dt_us = std::max<uint64_t>(1,
        static_cast<uint64_t>(std::llround(static_cast<double>(timeStep) * 1e6)));

    //////////////////////////// Sample wave height at origin for monitoring spectral params ////////////////////////////
    // if(waveHeights_pub_ && getOcean())
    // {
    //     // 4x4 grid, offsets chosen off any multiple of the 101 m / 893 m
    //     // cascade periods so points don't alias onto the same repeating pattern.
    //     static const std::array<double, 4> xs = {0.0, 137.0, 251.0, 389.0};
    //     static const std::array<double, 4> ys = {0.0, 149.0, 277.0, 401.0};

    //     std_msgs::msg::Float64MultiArray waveHeightsMsg;
    //     waveHeightsMsg.data.reserve(xs.size() * ys.size());

    //     auto* glOcean = getOcean()->getOpenGLOcean();
    //     for (double x : xs)
    //         for (double y : ys)
    //             waveHeightsMsg.data.push_back(glOcean->ComputeWaveHeight(x, y));

    //     waveHeights_pub_->publish(waveHeightsMsg);
    // }
    
    //////////////////////////// AUTHORITY MANAGEMENT ////////////////////////////
    {
    std::unique_lock<std::mutex> lk(authority_mutex);

    // (1) Settle: charge the step that just completed against every ACTIVE stepper.
    for (auto & [id, node] : authorityNodes_) {
        if (node.mode != AuthorityMode::STEPPER || !node.stepper_active_) continue;
        node.remaining_us_ = (node.remaining_us_ > dt_us) ? (node.remaining_us_ - dt_us) : 0;
        // A charged step only ran because it was affordable, so crossing below dt here
        // means we JUST entered starvation: start the watchdog grace now.
        if (node.remaining_us_ < dt_us)
            node.hold_started_ = std::chrono::steady_clock::now();
    }

    // (2) Gate the NEXT step: block while any authority still holds the sim.
    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto next_deadline = std::chrono::steady_clock::time_point::max();
        bool still_holding = false;

        for (auto & [id, node] : authorityNodes_) {
            if (node.mode == AuthorityMode::VOTER) {
                if (node.permissionToStep_) continue;              // not holding
                if (node.authority_timeout_ms > 0) {
                    auto deadline = node.hold_started_
                                  + std::chrono::milliseconds(node.authority_timeout_ms);
                    if (deadline <= now) {
                        RCLCPP_WARN(nh_->get_logger(),
                            "[sim_mgr] Voter '%s' (ID %ld) hold timed out after %ld ms, force-releasing",
                            node.node_name.c_str(), id, node.authority_timeout_ms);
                        node.permissionToStep_ = true;
                        continue;
                    }
                    if (deadline < next_deadline) next_deadline = deadline;
                }
                still_holding = true;
            } else { // STEPPER
                if (!node.stepper_active_) continue;               // inactive: does not constrain
                if (node.remaining_us_ >= dt_us) continue;         // can afford next step: not holding

                if (node.authority_timeout_ms > 0) {
                    auto deadline = node.hold_started_
                                  + std::chrono::milliseconds(node.authority_timeout_ms);
                    if (deadline <= now) {
                        RCLCPP_WARN(nh_->get_logger(),
                            "[sim_mgr] Stepper '%s' (ID %ld) starved for %ld ms, going inactive",
                            node.node_name.c_str(), id, node.authority_timeout_ms);
                        node.stepper_active_ = false;
                        node.remaining_us_   = 0;
                        continue;
                    }
                    if (deadline < next_deadline) next_deadline = deadline;
                }
                still_holding = true;
            }
        }

        if (!still_holding) break;

        if (next_deadline == std::chrono::steady_clock::time_point::max())
            authority_cv.wait(lk);
        else
            authority_cv.wait_until(lk, next_deadline);
    }
    }
	
    // NEW: Publishers now take sim time instead of sampling wall time 
    // Useful for syncing with use_sim_time for RTF scaled runs vvv

    ////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = getSensor(id++)) != nullptr)
    {
        if(!sensor->isNewDataAvailable())
            continue;

        if(sensor->getType() != SensorType::VISION)
        {
            if(pubs_.find(sensor->getName()) == pubs_.end())
                continue;

            switch(((ScalarSensor*)sensor)->getScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    interface_->PublishAccelerometer(pubs_.at(sensor->getName()), (Accelerometer*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::GYRO:
                    interface_->PublishGyroscope(pubs_.at(sensor->getName()), (Gyroscope*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::IMU:
                    interface_->PublishIMU(pubs_.at(sensor->getName()), (IMU*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::ODOM:
                    interface_->PublishOdometry(pubs_.at(sensor->getName()), (Odometry*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::DVL:
                {
                    interface_->PublishDVL(pubs_.at(sensor->getName()), (DVL*)sensor, currentSimTime_);
                    if(pubs_.find(sensor->getName() + "/altitude") != pubs_.end())
                        interface_->PublishDVLAltitude(pubs_.at(sensor->getName() + "/altitude"), (DVL*)sensor, currentSimTime_);
                }
                    break;

                case ScalarSensorType::INS:
                {
                    interface_->PublishINS(pubs_.at(sensor->getName()), (INS*)sensor, currentSimTime_);
                    if(pubs_.find(sensor->getName() + "/odometry") != pubs_.end())
                        interface_->PublishINSOdometry(pubs_.at(sensor->getName() + "/odometry"), (INS*)sensor, currentSimTime_);
                }
                    break;

                case ScalarSensorType::GPS:
                    interface_->PublishGPS(pubs_.at(sensor->getName()), (GPS*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::PRESSURE:
                    interface_->PublishPressure(pubs_.at(sensor->getName()), (Pressure*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::FT:
                    interface_->PublishForceTorque(pubs_.at(sensor->getName()), (ForceTorque*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::ENCODER:
                    interface_->PublishEncoder(pubs_.at(sensor->getName()), (RotaryEncoder*)sensor, currentSimTime_);
                    break;

                case ScalarSensorType::MULTIBEAM:
                {
                    interface_->PublishMultibeam(pubs_.at(sensor->getName()), (Multibeam*)sensor, currentSimTime_);
                    if(pubs_.find(sensor->getName() + "/pcl") != pubs_.end())
                        interface_->PublishMultibeamPCL(pubs_.at(sensor->getName() + "/pcl"), (Multibeam*)sensor, currentSimTime_);
                }
                    break;

                case ScalarSensorType::PROFILER:
                    interface_->PublishProfiler(pubs_.at(sensor->getName()), (Profiler*)sensor, currentSimTime_);
                    break;

                default:
                    break;
            }
        }

        sensor->MarkDataOld();
    }

    ///////////////////////////////////////COMMS///////////////////////////////////////////////////
    id = 0;
    Comm* comm;
    while((comm = getComm(id++)) != nullptr)
    {
        if(pubs_.find(comm->getName()) == pubs_.end())
            continue;

        switch(comm->getType())
        {
            case CommType::ACOUSTIC:
            {
                std_msgs::msg::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->getName())
                    )->publish(msg);
                }
            }
                break;

            case CommType::USBL:
            {
                if(comm->isNewDataAvailable())
                {
                    interface_->PublishUSBL(pubs_.at(comm->getName()), pubs_.at(comm->getName() + "/beacon_info"), (USBL*)comm, currentSimTime_);
                    comm->MarkDataOld();
                }

                std_msgs::msg::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->getName() + "/received_data")
                    )->publish(msg);
                }
            }
                break;

            case CommType::OPTICAL:
            {
                std_msgs::msg::Float64 msg;
                msg.data = ((OpticalModem*)comm)->getReceptionQuality();
                std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(
                    pubs_.at(comm->getName())
                )->publish(msg);

                std_msgs::msg::String msg2;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg2.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->getName() + "/received_data")
                    )->publish(msg2);
                }
            }
                break;

            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while((ent = getEntity(id++)) != nullptr)
    {
        if(ent->getType() == EntityType::ANIMATED)
        {
            if(pubs_.find(ent->getName() + "/odometry") == pubs_.end())
                continue;

            interface_->PublishTrajectoryState(pubs_.at(ent->getName() + "/odometry"), pubs_.at(ent->getName() + "/iteration"), (AnimatedEntity*)ent, currentSimTime_);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = getContact(id++)) != nullptr)
    {
        if(!cnt->isNewDataAvailable())
            continue;

        if(pubs_.find(cnt->getName()) != pubs_.end())
        {
            interface_->PublishContact(pubs_[cnt->getName()], cnt, currentSimTime_);
            cnt->MarkDataOld();
        }
    }

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->publishBaseLinkTransform_)
            interface_->PublishTF(tf_, rosRobots_[i]->robot_->getTransform(), currentSimTime_, "world_ned", rosRobots_[i]->robot_->getName() + "/base_link");
    }
    
    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/motors") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = currentSimTime_;
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::MOTOR)
                {
                    Motor* mtr = (Motor*)actuator;
                    msg.name.push_back(mtr->getJointName());
                    msg.position.push_back(mtr->getAngle());
                    msg.velocity.push_back(mtr->getAngularVelocity());
                    msg.effort.push_back(mtr->getTorque());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/motors")
                )->publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/servos") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = currentSimTime_;
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::SERVO)
                {
                    Servo* srv = (Servo*)actuator;
                    msg.name.push_back(srv->getJointName());
                    msg.position.push_back(srv->getPosition());
                    msg.velocity.push_back(srv->getVelocity());
                    msg.effort.push_back(srv->getEffort());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/servos")
                )->publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/rudders") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = currentSimTime_;
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::RUDDER)
                {
                    Rudder* rudder = (Rudder*)actuator;
                    msg.name.push_back(rudder->getName());
                    msg.position.push_back(rudder->getSetpoint());
                    msg.velocity.push_back(0.0);
                    msg.effort.push_back(0.0);
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/rudders")
                )->publish(msg);
        }

        if(rosRobots_[i]->thrusterSetpoints_.size() != 0
           && pubs_.find(rosRobots_[i]->robot_->getName() + "/thrusters") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros2::msg::ThrusterState msg;
            msg.header.stamp = currentSimTime_;
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            msg.setpoint.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.rpm.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.thrust.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.torque.resize(rosRobots_[i]->thrusterSetpoints_.size());

            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::THRUSTER)
                {
                    Thruster* th = (Thruster*)actuator;
                    msg.setpoint[thID] = th->getSetpoint();
                    msg.rpm[thID] = th->getOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;

                    if(thID == rosRobots_[i]->thrusterSetpoints_.size())
                        break;
                }
                else if(actuator->getType() == ActuatorType::SIMPLE_THRUSTER)
                {
                    SimpleThruster* th = (SimpleThruster*)actuator;
                    msg.setpoint[thID] = th->getThrustSetpoint();
                    msg.rpm[thID] = 0.0;
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;

                    if(thID == rosRobots_[i]->thrusterSetpoints_.size())
                        break;
                }
            }
            std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::ThrusterState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/thrusters")
                )->publish(msg);
        }

        if(rosRobots_[i]->rotatingElementsSetpoints_.size() != 0
           && pubs_.find(rosRobots_[i]->robot_->getName() + "/rotating_elements") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros2::msg::ThrusterState msg;
            msg.header.stamp = currentSimTime_;
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            msg.setpoint.resize(rosRobots_[i]->rotatingElementsSetpoints_.size());
            msg.rpm.resize(rosRobots_[i]->rotatingElementsSetpoints_.size());
            msg.thrust.resize(rosRobots_[i]->rotatingElementsSetpoints_.size());
            msg.torque.resize(rosRobots_[i]->rotatingElementsSetpoints_.size());

            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::ROTATING_ELEMENT)
                {
                    RotatingElement* re = (RotatingElement*)actuator;
                    msg.setpoint[thID] = re->getSetpoint();
                    msg.rpm[thID] = re->getOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = re->getThrust();
                    msg.torque[thID] = re->getTorque();
                    ++thID;

                    if(thID == rosRobots_[i]->rotatingElementsSetpoints_.size())
                        break;
                }
            }
            std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::ThrusterState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/rotating_elements")
                )->publish(msg);
        }

        if(rosRobots_[i]->controlSurfacesSetpoints_.size() != 0
           && pubs_.find(rosRobots_[i]->robot_->getName() + "/control_surfaces") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = currentSimTime_;
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            msg.position.resize(rosRobots_[i]->controlSurfacesSetpoints_.size());

            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::ROTATING_ELEMENT)
                {
                    ControlSurface* cs = (ControlSurface*)actuator;
                    msg.position[thID] = cs->getSetpoint();
                    ++thID;

                    if(thID == rosRobots_[i]->controlSurfacesSetpoints_.size())
                        break;
                }
            }
            std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/control_surfaces")
                )->publish(msg);
        }
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        unsigned int rudderID = 0;
	// New actuator elements: Check Stonefish Library
        unsigned int rotElemID = 0;
        unsigned int ctrlSurfID = 0;

        while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
        {
            switch(actuator->getType())
            {
                case ActuatorType::PUSH:
                {
                    Push* push = (Push*)actuator;    
                    if(rosRobots_[i]->thrusterSetpointsChanged_)
                    {
                        push->setForce(rosRobots_[i]->thrusterSetpoints_[thID++]);
                    }

                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = push->getName();
                        msg.wrench.force.x = push->getForce();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::THRUSTER:
                {
                    Thruster* th = ((Thruster*)actuator);
                    if(rosRobots_[i]->thrusterSetpointsChanged_)
                    {
                        th->setSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++]);
                    }
                        
                    auto it = pubs_.find(actuator->getName()+"/wrench");
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = th->getName();
                        msg.wrench.force.x = th->getThrust();
                        msg.wrench.torque.x = th->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }            

                    it = pubs_.find(actuator->getName()+"/joint_state");
                    if(it != pubs_.end())
                    {
                        //Publish propeller rotation for visualization
                        sensor_msgs::msg::JointState msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = th->getName();
                        msg.name.push_back(th->getName()+"/propeller");
                        msg.position.push_back(th->getAngle()/Scalar(100)); // Scaled for visualisation 
                        msg.velocity.push_back(th->getOmega());
                        msg.effort.push_back(th->getThrust());
                        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::SIMPLE_THRUSTER:
                {
                    SimpleThruster* th = ((SimpleThruster*)actuator);
                    if(rosRobots_[i]->thrusterSetpointsChanged_)
                    {
                        th->setSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++], Scalar(0));
                    }
                        
                    auto it = pubs_.find(actuator->getName()+"/wrench");
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = th->getName();
                        msg.wrench.force.x = th->getThrust();
                        msg.wrench.torque.x = th->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }            

                    it = pubs_.find(actuator->getName()+"/joint_state");
                    if(it != pubs_.end())
                    {
                        //Publish propeller rotation for visualization
                        sensor_msgs::msg::JointState msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = th->getName();
                        msg.name.push_back(th->getName()+"/propeller");
                        msg.position.push_back(th->getAngle());  
                        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::PROPELLER:
                {
                    Propeller* prop = (Propeller*)actuator;
                    if(rosRobots_[i]->propellerSetpointsChanged_)
                    {
                        prop->setSetpoint(rosRobots_[i]->propellerSetpoints_[propID++]);
                    }

                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = prop->getName();
                        msg.wrench.force.x = prop->getThrust();
                        msg.wrench.torque.x = prop->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::RUDDER:
                {
                    if(rosRobots_[i]->rudderSetpointsChanged_)
                    {
                        ((Rudder*)actuator)->setSetpoint(rosRobots_[i]->rudderSetpoints_[rudderID++]);
                    }
                }
                    break;

		// New actuator elements: Check Stonefish Library:

                case ActuatorType::ROTATING_ELEMENT:
                {
                    RotatingElement* re = (RotatingElement*)actuator;
                    if(rosRobots_[i]->rotatingElementsSetpointsChanged_)
                    {
                        re->setSetpoint(rosRobots_[i]->rotatingElementsSetpoints_[rotElemID++]);
                    }

                    auto it = pubs_.find(actuator->getName()+"/wrench");
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = re->getName();
                        msg.wrench.force.x = re->getThrust();
                        msg.wrench.torque.x = re->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }

                    it = pubs_.find(actuator->getName()+"/joint_state");
                    if(it != pubs_.end())
                    {
                        //Publish rotation for visualization
                        sensor_msgs::msg::JointState msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = re->getName();
                        msg.name.push_back(re->getName()+"/rotating_element");
                        msg.position.push_back(re->getAngle()/Scalar(100)); // Scaled for visualisation
                        msg.velocity.push_back(re->getOmega());
                        msg.effort.push_back(re->getThrust());
                        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::CONTROL_SURFACE:
                {
                    ControlSurface* cs = (ControlSurface*)actuator;
                    if(rosRobots_[i]->controlSurfacesSetpointsChanged_)
                    {
                        cs->setSetpoint(rosRobots_[i]->controlSurfacesSetpoints_[ctrlSurfID++]);
                    }

                    auto it = pubs_.find(actuator->getName()+"/joint_state");
                    if(it != pubs_.end())
                    {
                        //Publish deflection for visualization
                        sensor_msgs::msg::JointState msg;
                        msg.header.stamp = currentSimTime_;
                        msg.header.frame_id = cs->getName();
                        msg.name.push_back(cs->getName()+"/control_surface");
                        msg.position.push_back(cs->getAngle());
                        msg.velocity.push_back(0.0);
                        msg.effort.push_back(0.0);
                        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::MOTOR:
                {
                    if(rosRobots_[i]->servoSetpoints_.size() == 0)
                        continue;

                    auto it = rosRobots_[i]->servoSetpoints_.find(((Motor*)actuator)->getJointName());
                    if(it != rosRobots_[i]->servoSetpoints_.end())
                    {
                        if(it->second.first == ServoControlMode::TORQUE)
                        {
                            ((Motor*)actuator)->setCommand(it->second.second);
                        }
                    }
                }
                    break;

                case ActuatorType::SERVO:
                {
                    if(rosRobots_[i]->servoSetpoints_.size() == 0)
                        continue;

                    auto it = rosRobots_[i]->servoSetpoints_.find(((Servo*)actuator)->getJointName());
                    if(it != rosRobots_[i]->servoSetpoints_.end())
                    {
                        if(it->second.first == ServoControlMode::VELOCITY)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::VELOCITY);
                            ((Servo*)actuator)->setDesiredVelocity(it->second.second);
                        }
                        else if(it->second.first == ServoControlMode::POSITION)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::POSITION);
                            ((Servo*)actuator)->setDesiredPosition(it->second.second);
                        }
                    }
                }
                    break;

                case ActuatorType::VBS:
                {
                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        std_msgs::msg::Float64 msg;
                        msg.data = ((VariableBuoyancy*)actuator)->getLiquidVolume();
                        std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::SUCTION_CUP:
                {
                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        std_msgs::msg::Bool msg;
                        msg.data = ((SuctionCup*)actuator)->getPump();
                        std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(it->second)->publish(msg);
                    }
                }
                    break;

                default:
                    break;
            }
        }
        //Reset change flags
        rosRobots_[i]->thrusterSetpointsChanged_ = false;
        rosRobots_[i]->propellerSetpointsChanged_ = false;
        rosRobots_[i]->rudderSetpointsChanged_ = false;
        rosRobots_[i]->rotatingElementsSetpointsChanged_ = false;
        rosRobots_[i]->controlSurfacesSetpointsChanged_ = false;
    }

    /////////////////////////////////// RESPAWN REQUESTS ///////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->respawnRequested_)
        {
            rosRobots_[i]->robot_->Respawn(this, rosRobots_[i]->respawnOrigin_);
            rosRobots_[i]->respawnRequested_ = false;
        }
    }    

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/debug/physics") != pubs_.end())
        {
            Robot* r = rosRobots_[i]->robot_;
            size_t lID = 0;
            SolidEntity* link;
            stonefish_ros2::msg::DebugPhysics msg;
            msg.header.stamp = currentSimTime_;

            auto debugPub = std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::DebugPhysics>>(
                pubs_.at(r->getName() + "/debug/physics")
            );

            while((link = r->getLink(lID++)) != nullptr)
            {
                msg.header.frame_id = link->getName();
                
                msg.mass = link->getMass();
                msg.volume = link->getVolume();
                msg.surface = link->getSurface();
                Vector3 inertia = link->getInertia();
                msg.inertia.x = inertia.getX();
                msg.inertia.y = inertia.getY();
                msg.inertia.z = inertia.getZ();

                Vector3 cog = -link->getCG2OTransform().getOrigin();
                msg.cog.x = cog.getX();
                msg.cog.y = cog.getY();
                msg.cog.z = cog.getZ();
                
                Vector3 cob = link->getCG2OTransform() * link->getCB();
                msg.cob.x = cob.getX();
                msg.cob.y = cob.getY();
                msg.cob.z = cob.getZ();
                
                Matrix3 toOrigin = link->getOTransform().getBasis().inverse();

                Vector3 vel = toOrigin * link->getLinearVelocity();
                Vector3 avel = toOrigin * link->getAngularVelocity();
                msg.velocity.linear.x = vel.getX();
                msg.velocity.linear.y = vel.getY();
                msg.velocity.linear.z = vel.getZ();
                msg.velocity.angular.x = avel.getX();
                msg.velocity.angular.y = avel.getY();
                msg.velocity.angular.z = avel.getZ();

                Vector3 Fb, Tb, Fd, Td, Ff, Tf;
                link->getHydrodynamicForces(Fb, Tb, Fd, Td, Ff, Tf);            
                Vector3 Cd, Cf;
                link->getHydrodynamicCoefficients(Cd, Cf);
                msg.damping_coeff.x = Cd.getX();
                msg.damping_coeff.y = Cd.getY();
                msg.damping_coeff.z = Cd.getZ();
                msg.skin_friction_coeff.x = Cf.getX();
                msg.skin_friction_coeff.y = Cf.getY();
                msg.skin_friction_coeff.z = Cf.getZ();

                // Fb = toOrigin * Fb;
                // Tb = toOrigin * Tb;
                msg.buoyancy.force.x = Fb.getX();
                msg.buoyancy.force.y = Fb.getY();
                msg.buoyancy.force.z = Fb.getZ();
                msg.buoyancy.torque.x = Tb.getX();
                msg.buoyancy.torque.y = Tb.getY();
                msg.buoyancy.torque.z = Tb.getZ();

                Fd = toOrigin * Fd;
                Td = toOrigin * Td;
                msg.damping.force.x = Fd.getX();
                msg.damping.force.y = Fd.getY();
                msg.damping.force.z = Fd.getZ();
                msg.damping.torque.x = Td.getX();
                msg.damping.torque.y = Td.getY();
                msg.damping.torque.z = Td.getZ();

                Ff = toOrigin * Ff;
                Tf = toOrigin * Tf;
                msg.skin_friction.force.x = Ff.getX();
                msg.skin_friction.force.y = Ff.getY();
                msg.skin_friction.force.z = Ff.getZ();
                msg.skin_friction.torque.x = Tf.getX();
                msg.skin_friction.torque.y = Tf.getY();
                msg.skin_friction.torque.z = Tf.getZ();

                msg.wetted_surface = link->getWettedSurface();
                msg.submerged_volume = link->getSubmergedVolume();
                
                debugPub->publish(msg);
            }
        }
    }
}

void ROS2SimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = cameraMsgPrototypes_[cam->getName()].first;
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (uint8_t*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = cameraMsgPrototypes_[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
}

void ROS2SimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = cameraMsgPrototypes_[cam->getName()].first;
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = cameraMsgPrototypes_[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
}

void ROS2SimulationManager::ThermalCameraImageReady(ThermalCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->getName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->getDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
    imgPubs_.at(cam->getName()+"/display").publish(img2);
}

void ROS2SimulationManager::OpticalFlowCameraImageReady(OpticalFlowCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->getName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->getDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
    imgPubs_.at(cam->getName()+"/display").publish(img2);
}

void ROS2SimulationManager::SegmentationCameraImageReady(SegmentationCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (uint16_t*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->getName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->getDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
    imgPubs_.at(cam->getName()+"/display").publish(img2);
}

void ROS2SimulationManager::EventBasedCameraOutputReady(EventBasedCamera* cam)
{
    interface_->PublishEventBasedCamera(pubs_.at(cam->getName()), cam, currentSimTime_);
}

void ROS2SimulationManager::Multibeam2ScanReady(Multibeam2* mb)
{
    interface_->PublishMultibeam2(pubs_.at(mb->getName()), mb, currentSimTime_);
}

// NEW: Lidar implementation
void ROS2SimulationManager::LidarScanReady(Lidar* lidar)
{
    interface_->PublishLidar(pubs_.at(lidar->getName()), lidar, currentSimTime_);
}


void ROS2SimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[fls->getName()].first;
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (uint8_t*)fls->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[fls->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)fls->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(fls->getName()).publish(img);
    imgPubs_.at(fls->getName() + "/display").publish(disp);

}

void ROS2SimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[sss->getName()].first;
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (uint8_t*)sss->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[sss->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)sss->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(sss->getName()).publish(img);
    imgPubs_.at(sss->getName() + "/display").publish(disp);
}

void ROS2SimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[msis->getName()].first;
    img->header.stamp = currentSimTime_;
    memcpy(img->data.data(), (uint8_t*)msis->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[msis->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)msis->getDisplayDataPointer(), disp->step * disp->height);

    //Fill in the laser scan message
    Scalar currentAngle = (msis->getCurrentRotationStep() * msis->getRotationStepAngle()) * M_PI / 180.0;
    unsigned int currentBeamIndex = msis->getCurrentBeamIndex();

    sensor_msgs::msg::LaserScan laserscan;
    laserscan.header.stamp = img->header.stamp;
    laserscan.header.frame_id = msis->getName();
    laserscan.angle_min = currentAngle;
    laserscan.angle_max = currentAngle;
    laserscan.angle_increment = 0.0;
    laserscan.time_increment = 0.0;
    laserscan.range_min = msis->getRangeMin();
    laserscan.range_max = msis->getRangeMax();
    laserscan.ranges.resize(img->height);
    laserscan.intensities.resize(img->height);

    for(unsigned int i=0; i<img->height; ++i)
    {
        laserscan.ranges[i] = (laserscan.range_max - laserscan.range_min) * (img->height-1-i)/Scalar(img->height-1) + laserscan.range_min;
        laserscan.intensities[i] = img->data[img->step * i + currentBeamIndex];
    }

    //Publish messages
    imgPubs_.at(msis->getName()).publish(img);
    imgPubs_.at(msis->getName() + "/display").publish(disp);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pubs_.at(msis->getName() + "/beam"))->publish(laserscan);
}

void ROS2SimulationManager::EnableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                           std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    getOcean()->EnableCurrents();
    getAtmosphere()->EnableCurrents();
    res->message = "Ocean/Atmosphere current simulation enabled.";
    res->success = true;
}

void ROS2SimulationManager::DisableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                            std_srvs::srv::Trigger::Response::SharedPtr res)
{   
    (void)req;
    getOcean()->DisableCurrents();
    getAtmosphere()->DisableCurrents();
    res->message = "Ocean/Atmosphere current simulation disabled.";
    res->success = true;
}

void ROS2SimulationManager::RespawnRobotService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res)
{
    Vector3 p(req->origin.position.x, req->origin.position.y, req->origin.position.z);
    Quaternion q(req->origin.orientation.x, req->origin.orientation.y, req->origin.orientation.z, req->origin.orientation.w);
    Transform origin(q, p);
    
    if(RespawnROS2Robot(req->name, origin))
    {
        res->message = "Robot respawned.";
        res->success = true;
    }
    else    
    {
        res->message = "Robot not found.";
        res->success= false;
    }
}

// NEW: Respawn service for static objects 
void ROS2SimulationManager::RespawnStaticService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res)
{
    Vector3 p(req->origin.position.x, req->origin.position.y, req->origin.position.z);
    Quaternion q(req->origin.orientation.x, req->origin.orientation.y, req->origin.orientation.z, req->origin.orientation.w);
    Transform origin(q, p);
    
    std::vector<Entity*> entities = this->getEntities();
    
    for (auto it = entities.begin(); it != entities.end(); ++it) 
    {
        Entity* entity = *it;
        if (entity->getName() == req->name)
        {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Found entity " << entity->getName() << " to respawn.");
            // Physics cleanup
            if (entity->getType() == EntityType::STATIC) 
            {
                RespawnStaticEntity(static_cast<StaticEntity*>(entity), origin);
                res->message = "Respawning object: " + req->name + ".";
                res->success = true;
            }
            else
            {
                res->message = "Object " + req->name + " is not a static object.";
                res->success = false;
            }
            return; // Successfully respawned, exit function
        }
    }
    res->message = "Entity " + req->name + " not found.";
    res->success = false;
    return;
}

// NEW: Respawn service for dynamic objects 
void ROS2SimulationManager::RespawnDynamicService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res)
{
    Vector3 p(req->origin.position.x, req->origin.position.y, req->origin.position.z);
    Quaternion q(req->origin.orientation.x, req->origin.orientation.y, req->origin.orientation.z, req->origin.orientation.w);
    Transform origin(q, p);
    
    std::vector<Entity*> entities = this->getEntities();
    
    for (auto it = entities.begin(); it != entities.end(); ++it) 
    {
        Entity* entity = *it;
        if (entity->getName() == req->name)
        {
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Found entity " << entity->getName() << " to respawn.");
            // Physics cleanup
            if (entity->getType() == EntityType::SOLID) 
                RespawnSolidEntity(static_cast<SolidEntity*>(entity), origin);
            else
            {
                res->message = "Object " + req->name + " is not a dynamic object.";
                res->success = false;
            }
            return; // Successfully deleted, exit function
        }
    }
    res->message = "Entity " + req->name + " not found.";
    res->success = false;
}

// NEW: RTF service for changing RTF dynamically 
void ROS2SimulationManager::setRTFService(const stonefish_ros2::srv::SetRTF::Request::SharedPtr req,
                             stonefish_ros2::srv::SetRTF::Response::SharedPtr res)
{
    double rtf = req->real_time_factor;
    if (rtf < 0.0)
    {
        res->message = "RTF must be non-negative.";
        res->success = false;
        return;
    }
    this->setRealtimeFactor(rtf);
    customRTF = rtf;
    res->message = "Real-time factor set to " + std::to_string(rtf) + ".";
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Real-time factor set to " << rtf << ".");
    res->success = true;
}

// NEW: Ocean service for changing Ocean entity dynamically
void ROS2SimulationManager::changeOceanService(const stonefish_ros2::srv::ChangeOcean::Request::SharedPtr req,
                                    stonefish_ros2::srv::ChangeOcean::Response::SharedPtr res)
{
    double  wind_speed = req->wind_speed;
    double  wind_direction = req->direction;
    double  wave_age = req->age;

    res->success = getOcean()->UpdateOceanData(wind_speed, wind_direction*2*M_PI/360.0, wave_age);
    if(res->success)
    {
        res->message = "Ocean parameters updated.";
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Ocean parameters updated: wind speed = " << wind_speed << " m/s, wind direction = " << wind_direction << " degrees, wave age = " << wave_age << ".");
    }
    else
    {
        res->message = "Failed to update ocean parameters. Please check the input values.";
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Failed to update ocean parameters. Please check the input values: wind speed = " << wind_speed << " m/s, wind direction = " << wind_direction << " degrees, wave age = " << wave_age << ".");
    }
    return;
}

// NEW: Fog service for changing Atmospheric Fog dynamically
void ROS2SimulationManager::changeFogService(const stonefish_ros2::srv::ChangeFog::Request::SharedPtr req,
                                stonefish_ros2::srv::ChangeFog::Response::SharedPtr res)
{
    // Clamp density and track if bounded
    bool bounded = false;
    Scalar density = req->density;
    if (density < 0.0) { density = 0.0; bounded = true; }
    else if (density > 1.0) { density = 1.0; bounded = true; }

    // Handle color: if (0,0,0), retrieve current
    Vector3 color(req->color.x, req->color.y, req->color.z);
    std::string msg = "Fog updated";

    if (color.norm() < 1e-6) {
        auto current_color = getAtmosphere()->getOpenGLAtmosphere()->GetFogColor();
        color.setX(current_color.r);
        color.setY(current_color.g);
        color.setZ(current_color.b);
        msg += " (using current color)";
    }

    getAtmosphere()->SetFog(density, color);

    if (bounded) msg += " (density clamped to [0, 1])";
    
    res->message = msg;
    res->success = true;
}

// NEW: Respawn service for dynamic objects 
void ROS2SimulationManager::DynamicSpawnService(const stonefish_ros2::srv::SpawnObject::Request::SharedPtr req, 
                                stonefish_ros2::srv::SpawnObject::Response::SharedPtr res)
{
    std::string filepath = dataPath_ + req->path;
    std::string filename = req->name;
    ROS2ScenarioParser parser(this, nh_);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Parsing scenario file for static spawn: " << filename.c_str() << " from " << filepath.c_str());

    // Load the XML file
    tinyxml2::XMLDocument doc;
    XMLError result = doc.LoadFile(filepath.c_str());
    if(result != XML_SUCCESS)
    {
        switch(result)
        {
            case XMLError::XML_ERROR_FILE_NOT_FOUND:
            {
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Scenario parser: File not found!");
            }
                break;

            default:
            {
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Scenario parser: Syntax error in file!");
            }
                break;
        }
        res->message = "Failed to load scenario file.";
        res->success = false;
        return;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if(root == nullptr)
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Scenario parser: Empty scenario file!");
        res->message = "Empty scenario file or <scenario> tag not found!";
        res->success = false;
        return;
    }
    
    tinyxml2::XMLElement* staticElem = root->FirstChildElement("dynamic");
    if (!staticElem)
    {
        RCLCPP_ERROR(nh_->get_logger(), "No <dynamic> tag found in %s", filename.c_str());
        res->message = "Invalid scenario: missing <dynamic> tag.";
        res->success = false;
        return;
    }

    staticElem->SetAttribute("name", filename.c_str());
    RCLCPP_INFO(nh_->get_logger(), "Overrode name.");

    tinyxml2::XMLElement* transformElem = staticElem->FirstChildElement("world_transform");
    if (!transformElem)
    {
        // It doesn't exist, so create a new one and append it to <static>
        transformElem = doc.NewElement("world_transform");
        staticElem->InsertEndChild(transformElem);
        RCLCPP_INFO(nh_->get_logger(), "world_transform not found, creating new element.");
    }
    if (transformElem)
    {
        // Format strings
        std::string xyz = std::to_string(req->xyz.x) + " " +
                          std::to_string(req->xyz.y) + " " +
                          std::to_string(req->xyz.z);
        
        std::string rpy = std::to_string(req->rpy.x) + " " +
                          std::to_string(req->rpy.y) + " " + 
                          std::to_string(req->rpy.z);

        // Modify the XML in memory
        transformElem->SetAttribute("xyz", xyz.c_str());
        transformElem->SetAttribute("rpy", rpy.c_str());
        
        RCLCPP_INFO(nh_->get_logger(), "Overrode world_transform with request origin.");
    }

    if (parser.ParseDynamic(staticElem))
    {
        res->message = "Successfully spawned dynamic object at request origin.";
        res->success = true;
    }
    else
    {
        res->message = "Failed to parse dynamic element details.";
        res->success = false;
    }
}

// NEW: Spawn service for static objects 
void ROS2SimulationManager::StaticSpawnService(const stonefish_ros2::srv::SpawnObject::Request::SharedPtr req, 
                                stonefish_ros2::srv::SpawnObject::Response::SharedPtr res)
{
    std::string filepath = dataPath_ + req->path;
    std::string filename = req->name;
    ROS2ScenarioParser parser(this, nh_);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Parsing scenario file for static spawn: " << filename.c_str() << " from " << filepath.c_str());

    // Load the XML file
    tinyxml2::XMLDocument doc;
    XMLError result = doc.LoadFile(filepath.c_str());
    if(result != XML_SUCCESS)
    {
        switch(result)
        {
            case XMLError::XML_ERROR_FILE_NOT_FOUND:
            {
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Scenario parser: File not found!");
            }
                break;

            default:
            {
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Scenario parser: Syntax error in file!");
            }
                break;
        }
        res->message = "Failed to load scenario file.";
        res->success = false;
        return;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if(root == nullptr)
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Scenario parser: Empty scenario file!");
        res->message = "Empty scenario file or <scenario> tag not found!";
        res->success = false;
        return;
    }
    
    tinyxml2::XMLElement* staticElem = root->FirstChildElement("static");
    if (!staticElem)
    {
        RCLCPP_ERROR(nh_->get_logger(), "No <static> tag found in %s", filename.c_str());
        res->message = "Invalid scenario: missing <static> tag.";
        res->success = false;
        return;
    }

    staticElem->SetAttribute("name", filename.c_str());
    RCLCPP_INFO(nh_->get_logger(), "Overrode name.");

    tinyxml2::XMLElement* transformElem = staticElem->FirstChildElement("world_transform");
    if (!transformElem)
    {
        // It doesn't exist, so create a new one and append it to <static>
        transformElem = doc.NewElement("world_transform");
        staticElem->InsertEndChild(transformElem);
        RCLCPP_INFO(nh_->get_logger(), "world_transform not found, creating new element.");
    }
    if (transformElem)
    {
        // Format strings
        std::string xyz = std::to_string(req->xyz.x) + " " +
                          std::to_string(req->xyz.y) + " " +
                          std::to_string(req->xyz.z);
        
        std::string rpy = std::to_string(req->rpy.x) + " " +
                          std::to_string(req->rpy.y) + " " + 
                          std::to_string(req->rpy.z);

        // Modify the XML in memory
        transformElem->SetAttribute("xyz", xyz.c_str());
        transformElem->SetAttribute("rpy", rpy.c_str());
        
        RCLCPP_INFO(nh_->get_logger(), "Overrode world_transform with request origin.");
    }

    if (parser.ParseStatic(staticElem))
    {
        res->message = "Successfully spawned static object at request origin.";
        res->success = true;
    }
    else
    {
        res->message = "Failed to parse static element details.";
        res->success = false;
    }
}

// NEW: Spawn service for Robots (this works well, deletion breaks sim)
// A bit more complicated than satic/dynamic spawn, needs queues to ensure synchronicity
// between processes
void ROS2SimulationManager::RobotSpawnService(const stonefish_ros2::srv::SpawnObject::Request::SharedPtr req,
                                stonefish_ros2::srv::SpawnObject::Response::SharedPtr res)
{
    if(getRobot(req->name) != nullptr)
    {
        res->message = "A robot named '" + req->name + "' already exists.";
        res->success = false;
        return;
    }

    std::string filepath = dataPath_ + req->path;
    auto doc = std::make_shared<tinyxml2::XMLDocument>();
    if(doc->LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        res->message = "Failed to load scenario file: " + filepath;
        res->success = false;
        return;
    }
    tinyxml2::XMLElement* root = doc->RootElement();
    tinyxml2::XMLElement* robotElem = root ? root->FirstChildElement("robot") : nullptr;
    if(!robotElem)
    {
        res->message = "Invalid scenario: missing <scenario>/<robot>.";
        res->success = false;
        return;
    }

    robotElem->SetAttribute("name", req->name.c_str());
    tinyxml2::XMLElement* tf = robotElem->FirstChildElement("world_transform");
    if(!tf) { tf = doc->NewElement("world_transform"); robotElem->InsertEndChild(tf); }
    tf->SetAttribute("xyz", (std::to_string(req->xyz.x)+" "+std::to_string(req->xyz.y)+" "+std::to_string(req->xyz.z)).c_str());
    tf->SetAttribute("rpy", (std::to_string(req->rpy.x)+" "+std::to_string(req->rpy.y)+" "+std::to_string(req->rpy.z)).c_str());

    auto result = std::make_shared<std::promise<std::pair<bool,std::string>>>();
    auto future = result->get_future();
    {
        std::lock_guard<std::mutex> lk(spawnQueueMutex_);
        robotSpawnQueue_.push_back({doc, robotElem, req->name, result});
    }

    // res->message = "Spawn worked.";
    // res->success = true;
    // return;

    if(future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        res->message = "Spawn timed out (is the simulation stepping?).";
        res->success = false;
        return;
    }

    auto outcome = future.get();
    res->success = outcome.first;
    res->message = outcome.second;
} 
void ROS2SimulationManager::DrainRobotSpawnQueue()
{
    std::vector<RobotSpawnRequest> pending;
    {
        std::lock_guard<std::mutex> lk(spawnQueueMutex_);
        if(robotSpawnQueue_.empty()) return;
        pending.swap(robotSpawnQueue_);
    }
    for(auto& r : pending)
    {
        ROS2ScenarioParser parser(this, nh_);
        bool ok = parser.ParseRobot(r.robotElem);   // safe: sim thread, no step in flight
        r.result->set_value({ok, ok ? ("Successfully spawned robot '" + r.name + "'.")
                                     : ("Failed to parse robot '" + r.name + "'.")});
    }
}

// NEW: Robot deletion not working.
void ROS2SimulationManager::DrainRobotDeleteQueue()
{
    std::vector<RobotDeleteRequest> pending;
    {
        std::lock_guard<std::mutex> lk(deleteQueueMutex_);
        if(robotDeleteQueue_.empty()) return;
        pending.swap(robotDeleteQueue_);
    }
    for(auto& d : pending)
    {
        Robot* robot = getRobot(d.name);
        if(robot == nullptr) { d.result->set_value({false, "Robot '"+d.name+"' not found."}); continue; }

        std::shared_ptr<ROS2Robot> rr;
        for(auto it=rosRobots_.begin(); it!=rosRobots_.end(); ++it)
            if((*it)->robot_ == robot) { rr=*it; rosRobots_.erase(it); break; }
        if(rr)
        {
            for(const auto& k : rr->imgPubKeys_){ auto it=imgPubs_.find(k); if(it!=imgPubs_.end()){ it->second.shutdown(); imgPubs_.erase(it);} }
            for(const auto& k : rr->pubKeys_) pubs_.erase(k);
            for(const auto& k : rr->subKeys_) subs_.erase(k);
            for(const auto& k : rr->srvKeys_) srvs_.erase(k);
            for(const auto& k : rr->cameraProtoKeys_) cameraMsgPrototypes_.erase(k);
            for(const auto& k : rr->dualCameraProtoKeys_) dualImageCameraMsgPrototypes_.erase(k);
            for(const auto& k : rr->sonarProtoKeys_) sonarMsgPrototypes_.erase(k);
        }
        bool ok = RemoveRobot(robot);
        d.result->set_value({ok, ok ? ("Deleted robot '"+d.name+"'.") : ("RemoveRobot failed for '"+d.name+"'.")});
    }
}

// NEW: Delete objects from sim dynamically (Robots not working here)
void ROS2SimulationManager::DeleteObjectService(const stonefish_ros2::srv::DeleteObject::Request::SharedPtr req,
                                stonefish_ros2::srv::DeleteObject::Response::SharedPtr res)
{
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Delete request for '" << req->object_name << "'.");

    Robot* robot = getRobot(req->object_name);
    if(robot != nullptr)
    {
        auto result = std::make_shared<std::promise<std::pair<bool,std::string>>>();
        auto future = result->get_future();
        { std::lock_guard<std::mutex> lk(deleteQueueMutex_); robotDeleteQueue_.push_back({req->object_name, result}); }
        res->success = true; res->message = "Deleted robot '" + req->object_name + "'.";
        return;
    }

    // --- existing solid/static path unchanged ---
    std::vector<Entity*> entities = this->getEntities();
    for(auto it = entities.begin(); it != entities.end(); ++it)
    {
        Entity* entity = *it;
        if(entity->getName() == req->object_name)
        {
            if(entity->getType() == EntityType::SOLID)  RemoveSolidEntity(static_cast<SolidEntity*>(entity));
            if(entity->getType() == EntityType::STATIC) RemoveStaticEntity(static_cast<StaticEntity*>(entity));
            removeEntityFromList(entity);
            delete entity;
            res->message = "Successfully found " + req->object_name + " and deleted.";
            res->success = true;
            return;
        }
    }
    res->message = "Object " + req->object_name + " not found.";
    res->success = false;
}

// NEW: Uniform current subscriber callback 
void ROS2SimulationManager::UniformVFCallback(const stonefish_ros2::msg::Uniform::SharedPtr msg, Uniform* vf)
{
    vf->setVelocity(Vector3(msg->flow.x, msg->flow.y, msg->flow.z));   
    if (msg->turbulence) 
    {
        vf->enableTurbulence();
        TurbulenceMixer::Params _t;
        _t.strength= msg->t.strength;
        _t.scale= msg->t.scale;
        _t.frequency= msg->t.frequency;
        _t.persistence= msg->t.persistence;
        vf->m_turbulence.setParams(_t);
    }
    else vf->disableTurbulence();
    if (msg->gusts)
    {
        vf->enableGust();
        GustMixer::Params _g;
        _g.strength = msg->g.strength;
        _g.radius = msg->g.radius;   
        _g.spacing = msg->g.spacing;   
        _g.period = msg->g.period;    
        _g.duration = msg->g.duration; 
        vf->m_gust.setParams(_g);
    }
    else vf->disableGust();
}

void ROS2SimulationManager::JetVFCallback(const std_msgs::msg::Float64::SharedPtr msg, Jet* vf)
{
    vf->setOutletVelocity(msg->data);
}

void ROS2SimulationManager::ActuatorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Actuator* act)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(act->getType())
    {
        case ActuatorType::PUSH:
        case ActuatorType::SIMPLE_THRUSTER:
        case ActuatorType::THRUSTER:
        case ActuatorType::PROPELLER:
        case ActuatorType::VBS:
        case ActuatorType::LIGHT:
            ((LinkActuator*)act)->setRelativeActuatorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of actuator '" << act->getName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::TrajectoryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, ManualTrajectory* tr)
{
    Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Vector3 omega(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    tr->setTransform(Transform(q, p));
    tr->setLinearVelocity(v);
    tr->setAngularVelocity(omega);
}

void ROS2SimulationManager::SimpleThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, SimpleThruster* th)
{
    th->setSetpoint(msg->data, Scalar(0));
}

void ROS2SimulationManager::ThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, Thruster* th)
{
    th->setSetpoint(msg->data);
}

void ROS2SimulationManager::PropellerCallback(const std_msgs::msg::Float64::SharedPtr msg, Propeller* prop)
{
    prop->setSetpoint(msg->data);
}

void ROS2SimulationManager::PushCallback(const std_msgs::msg::Float64::SharedPtr msg, Push* push)
{
    push->setForce(msg->data);
}

void ROS2SimulationManager::VBSCallback(const std_msgs::msg::Float64::SharedPtr msg, VariableBuoyancy* act)
{
    act->setFlowRate(msg->data);
}

void ROS2SimulationManager::CommCallback(const std_msgs::msg::String::SharedPtr msg, Comm* comm)
{
    comm->SendMessage(msg->data);
}

void ROS2SimulationManager::SuctionCupService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, SuctionCup* suction)
{
    suction->setPump(req->data);
    if(req->data)
        res->message = "Pump turned on.";
    else 
        res->message = "Pump turned off.";
    res->success = true;
}

void ROS2SimulationManager::SensorService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                        std_srvs::srv::SetBool::Response::SharedPtr res, Sensor* sens)
{
    sens->setEnabled(req->data);
    if(req->data)
        res->message = "Sensor turned on.";
    else
        res->message = "Sensor turned off.";
    res->success = true;    
}

void ROS2SimulationManager::SensorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Sensor* sens)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens->getType())
    {
        case SensorType::LINK:
            ((LinkSensor*)sens)->setRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            ((VisionSensor*)sens)->setRelativeSensorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of sensor '" << sens->getName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::FLSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, FLS* fls)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        fls->setRangeMax(req->range_max);
        fls->setRangeMin(req->range_min);
        fls->setGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::SSSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, SSS* sss)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        sss->setRangeMax(req->range_max);
        sss->setRangeMin(req->range_min);
        sss->setGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::MSISService(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res, MSIS* msis)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0
       || req->range_min >= req->range_max
       || req->rotation_min < -180.0
       || req->rotation_max > 180.0
       || req->rotation_min >= req->rotation_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        msis->setRangeMax(req->range_max);
        msis->setRangeMin(req->range_min);
        msis->setGain(req->gain);
        msis->setRotationLimits(req->rotation_min, req->rotation_max);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::ThrustersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->thrusterSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of thruster setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->thrusterSetpoints_.size(); ++i)
        robot->thrusterSetpoints_[i] = msg->data[i];
    robot->thrusterSetpointsChanged_ = true;
}

void ROS2SimulationManager::PropellersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->propellerSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of propeller setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->propellerSetpoints_.size(); ++i)
        robot->propellerSetpoints_[i] = msg->data[i];
    robot->propellerSetpointsChanged_ = true;
}

void ROS2SimulationManager::RuddersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->rudderSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of rudder setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->rudderSetpoints_.size(); ++i)
        robot->rudderSetpoints_[i] = msg->data[i];
    robot->rudderSetpointsChanged_ = true;
}

// NEW: New actuator types with per medium models and sampling
void ROS2SimulationManager::RotatingElementsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->rotatingElementsSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of rotating_elements setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->rotatingElementsSetpoints_.size(); ++i)
        robot->rotatingElementsSetpoints_[i] = msg->data[i];
    robot->rotatingElementsSetpointsChanged_ = true;
}

void ROS2SimulationManager::ControlSurfacesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->controlSurfacesSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of control_elements setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->controlSurfacesSetpoints_.size(); ++i)
        robot->controlSurfacesSetpoints_[i] = msg->data[i];
    robot->controlSurfacesSetpointsChanged_ = true;
}


void ROS2SimulationManager::ServosCallback(const sensor_msgs::msg::JointState::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->name.size() == 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Desired joint state message is missing joint names!");
        return;
    }

    if(msg->position.size() > 0)
    {
        for(size_t i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::POSITION, (Scalar)msg->position[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->velocity.size() > 0)
    {
        for(size_t i=0; i<msg->velocity.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::VELOCITY, (Scalar)msg->velocity[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->effort.size() > 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "No effort control mode implemented in simulation!");
    }
}

void ROS2SimulationManager::JointCallback(const std_msgs::msg::Float64::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::string& jointName)
{
    try
    {
        robot->servoSetpoints_.at(jointName) = std::make_pair(mode, (Scalar)msg->data);
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointName);
    }
}

void ROS2SimulationManager::JointGroupCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::vector<std::string>& jointNames)
{
    if(msg->data.size() != jointNames.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong size of joint group message! Required: " << jointNames.size() << " Received: " << msg->data.size());
        return;
    }
    for(size_t i=0; i<jointNames.size(); ++i)
    {
        try
        {
            robot->servoSetpoints_.at(jointNames[i]) = std::make_pair(mode, (Scalar)msg->data[i]);
        }
        catch(const std::out_of_range& e)
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointNames[i]);
        }
    }
}

void ROS2SimulationManager::GlueService(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res, FixedJoint* fix)
{
    if(req->data)
    {
        fix->RemoveFromSimulation(this);
        fix->UpdateDefinition();
        fix->AddToSimulation(this);
        res->message = "Glue activated.";
    }
    else
    {
        fix->RemoveFromSimulation(this);  
        res->message = "Glue deactivated.";
    }
    res->success = true;
}

void ROS2SimulationManager::LightService(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res, Light* light)
{
    light->Switch(req->data);
    if(req->data)
        res->message = "Light turned on.";
    else
        res->message = "Light turned off.";
    res->success = true;
}

}
