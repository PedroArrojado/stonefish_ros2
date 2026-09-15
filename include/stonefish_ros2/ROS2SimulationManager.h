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
//  ROS2SimulationManager.h
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2025 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROS2SimulationManager__
#define __Stonefish_ROS2SimulationManager__

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "stonefish_ros2/msg/uniform.hpp"
#include "stonefish_ros2/srv/sonar_settings.hpp"
#include "stonefish_ros2/srv/sonar_settings2.hpp"
#include "stonefish_ros2/srv/respawn.hpp"
#include "stonefish_ros2/srv/set_rtf.hpp"
#include "stonefish_ros2/srv/change_ocean.hpp"
#include "stonefish_ros2/srv/change_fog.hpp"
#include "stonefish_ros2/srv/delete_object.hpp"
#include "stonefish_ros2/srv/spawn_object.hpp"
#include "stonefish_ros2/srv/apply_wrench.hpp"

#include "stonefish_ros2/srv/authority_node_sign_in.hpp"
#include "stonefish_ros2/srv/authority_node_sign_out.hpp"
#include "stonefish_ros2/msg/authority_node_vote.hpp"
#include "stonefish_ros2/msg/authority_node_step.hpp"

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Servo.h>
#include <map>

#include "rosgraph_msgs/msg/clock.hpp"

#include <future>

namespace tinyxml2 { class XMLDocument; class XMLElement; }

namespace sf
{
    class ROS2Interface;
    class Uniform;
    class Jet;
    class ManualTrajectory;
    class SimpleThruster;
    class Thruster;
    class Propeller;
    class Push;
    class VariableBuoyancy;
    class SuctionCup;
    class ColorCamera;
	class DepthCamera;
    class ThermalCamera;
    class OpticalFlowCamera;
    class SegmentationCamera;
    class EventBasedCamera;
    class Multibeam2;
    class Lidar;
    class FLS;
    class SSS;
    class MSIS;
    class FixedJoint;
    class Light;
    class Comm;

    struct ROS2Robot
	{
		Robot* robot_;
		bool publishBaseLinkTransform_;
		std::vector<Scalar> thrusterSetpoints_;
		std::vector<Scalar> propellerSetpoints_;
		std::vector<Scalar> rudderSetpoints_;
        std::vector<Scalar> rotatingElementsSetpoints_;
        std::vector<Scalar> controlSurfacesSetpoints_;
		std::map<std::string, std::pair<ServoControlMode, Scalar>> servoSetpoints_;
        bool thrusterSetpointsChanged_;
        bool propellerSetpointsChanged_;
        bool rudderSetpointsChanged_;
        bool rotatingElementsSetpointsChanged_;
        bool controlSurfacesSetpointsChanged_;
        bool respawnRequested_;
        Transform respawnOrigin_;

        std::vector<std::string> pubKeys_;
        std::vector<std::string> subKeys_;
        std::vector<std::string> srvKeys_;
        std::vector<std::string> imgPubKeys_;
        std::vector<std::string> cameraProtoKeys_;
        std::vector<std::string> dualCameraProtoKeys_;
        std::vector<std::string> sonarProtoKeys_;

		ROS2Robot(Robot* robot, unsigned int nThrusters, unsigned int nPropellers, unsigned int nRudders,
                unsigned int nRotatingElements, unsigned int nControlSurfaces)
			: robot_(robot), publishBaseLinkTransform_(false), thrusterSetpointsChanged_(false), 
            propellerSetpointsChanged_(false), rudderSetpointsChanged_(false), respawnRequested_(false)
		{
			thrusterSetpoints_ = std::vector<Scalar>(nThrusters, Scalar(0));
			propellerSetpoints_ = std::vector<Scalar>(nPropellers, Scalar(0));
			rudderSetpoints_ = std::vector<Scalar>(nRudders, Scalar(0));
            rotatingElementsSetpoints_ = std::vector<Scalar>(nRotatingElements, Scalar(0));
            controlSurfacesSetpoints_ = std::vector<Scalar>(nControlSurfaces, Scalar(0));
		};
	};

    enum class AuthorityMode : uint8_t { VOTER = 0, STEPPER = 1 };

    struct AuthorityNode
    {
        std::string node_name;
        int64_t authority_timeout_ms;
        int64_t authorityID;
        AuthorityMode mode = AuthorityMode::VOTER;

        // VOTER state (existing stop/continue gate)
        bool permissionToStep_ = true;

        // STEPPER state (sim-time budget / lockstep)
        uint64_t remaining_us_   = 0;      // sim-time credit not yet consumed
        bool     stepper_active_ = false;  // false after a watchdog timeout, until the next grant

        // Shared: wall-clock instant a hold/starvation began (watchdog reference)
        std::chrono::steady_clock::time_point hold_started_;
    };

    class ROS2SimulationManager : public SimulationManager
    {
    public:
        ROS2SimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath, std::string dataPath, const std::shared_ptr<rclcpp::Node>& nh);
        virtual ~ROS2SimulationManager();
		
        virtual void BuildScenario();
        virtual void DestroyScenario();
		virtual void SimulationStepCompleted(Scalar timeStep);

        virtual void ColorCameraImageReady(ColorCamera* cam);
	    virtual void DepthCameraImageReady(DepthCamera* cam);
        virtual void ThermalCameraImageReady(ThermalCamera* cam);
        virtual void OpticalFlowCameraImageReady(OpticalFlowCamera* cam);
        virtual void SegmentationCameraImageReady(SegmentationCamera* cam);
        virtual void EventBasedCameraOutputReady(EventBasedCamera* cam);
		virtual void Multibeam2ScanReady(Multibeam2* mb);
        virtual void LidarScanReady(Lidar* lidar);
		virtual void FLSScanReady(FLS* fls);
		virtual void SSSScanReady(SSS* sss);
		virtual void MSISScanReady(MSIS* msis);

        void AddROS2Robot(const std::shared_ptr<ROS2Robot>& robot);
        bool RespawnROS2Robot(const std::string& robotName, const Transform& origin);

        bool HasPendingStructuralChanges() override;
        void ApplyStructuralChanges() override;

        void SimulationClockSleep(uint64_t us);
        uint64_t getSimulationClock() const;
        std::map<std::string, rclcpp::ServiceBase::SharedPtr>& getServices();
        std::map<std::string, rclcpp::PublisherBase::SharedPtr>& getPublishers();
        std::map<std::string, image_transport::Publisher>& getImagePublishers();
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& getSubscribers();
        std::shared_ptr<image_transport::ImageTransport> getImageTransportHandle();
        std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& getCameraMsgPrototypes();
        std::map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& getDualImageCameraMsgPrototypes();
		std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& getSonarMsgPrototypes();        

        // Sim parameters services. Change environemnt without restart
        void setRTFService(const stonefish_ros2::srv::SetRTF::Request::SharedPtr req,
                             stonefish_ros2::srv::SetRTF::Response::SharedPtr res);
        void EnableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                            std_srvs::srv::Trigger::Response::SharedPtr res);
		void DisableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                             std_srvs::srv::Trigger::Response::SharedPtr res);
        void changeOceanService(const stonefish_ros2::srv::ChangeOcean::Request::SharedPtr req,
                                stonefish_ros2::srv::ChangeOcean::Response::SharedPtr res);
        void changeFogService(const stonefish_ros2::srv::ChangeFog::Request::SharedPtr req,
                                stonefish_ros2::srv::ChangeFog::Response::SharedPtr res);
        /*TODO: 
            jerkov water service
            weather services:
                rain?
                atmosphere (wind, temperature, pressure) - check current implementations
        */

        // Respawn services (equivalent to instant move)
        void RespawnRobotService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res);
        void RespawnStaticService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res);
        void RespawnDynamicService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res);
        
        // Spawning objects dynamically. If object exists, an int will be appended to the name (maybe this isnt good, better to return false?)
        void DynamicSpawnService(const stonefish_ros2::srv::SpawnObject::Request::SharedPtr req, 
                                stonefish_ros2::srv::SpawnObject::Response::SharedPtr res);
        void StaticSpawnService(const stonefish_ros2::srv::SpawnObject::Request::SharedPtr req, 
                                stonefish_ros2::srv::SpawnObject::Response::SharedPtr res);
        void RobotSpawnService(const stonefish_ros2::srv::SpawnObject::Request::SharedPtr req, 
                                stonefish_ros2::srv::SpawnObject::Response::SharedPtr res); // Not Implemented Yet.
        
        // Delete objects dynamically. Returns false if object didnt exist
        void DeleteObjectService(const stonefish_ros2::srv::DeleteObject::Request::SharedPtr req, 
                                stonefish_ros2::srv::DeleteObject::Response::SharedPtr res); 

        // Apply wrenches to objects. Returns false if object didnt exist or doesnt have a physics body.
        void ApplyWrenchService(const stonefish_ros2::srv::ApplyWrench::Request::SharedPtr req, 
                                stonefish_ros2::srv::ApplyWrench::Response::SharedPtr res);
        
        // Authority node management. Nodes that want to control the simulation (e.g. for stepping) must sign in and receive an authority ID and sign out when they are done.
        // Authority nodes can vote to stop and advance the sim on request. Votes to advance must be unanimous.
        int64_t AuthorityNodeSignIn(stonefish_ros2::srv::AuthorityNodeSignIn::Request::SharedPtr req, 
                             stonefish_ros2::srv::AuthorityNodeSignIn::Response::SharedPtr res);

        bool AuthorityNodeSignOut(stonefish_ros2::srv::AuthorityNodeSignOut::Request::SharedPtr req, 
                             stonefish_ros2::srv::AuthorityNodeSignOut::Response::SharedPtr res);

        void AuthorityNodeStepVote(const stonefish_ros2::msg::AuthorityNodeVote::SharedPtr vote);
        void AuthorityNodeStepGrant(const stonefish_ros2::msg::AuthorityNodeStep::SharedPtr grant);

        // Sensors and actuators callbacks
        void UniformVFCallback(const stonefish_ros2::msg::Uniform::SharedPtr msg, Uniform* vf);
        void JetVFCallback(const std_msgs::msg::Float64::SharedPtr msg, Jet* vf);
        void ActuatorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Actuator* act);
        void TrajectoryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, ManualTrajectory* tr);
        void PushCallback(const std_msgs::msg::Float64::SharedPtr msg, Push* push);
        void SimpleThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, SimpleThruster* th);
        void ThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, Thruster* th);
        void PropellerCallback(const std_msgs::msg::Float64::SharedPtr msg, Propeller* prop);
        void VBSCallback(const std_msgs::msg::Float64::SharedPtr msg, VariableBuoyancy* act);
        void SuctionCupService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, SuctionCup* suction);
        void SensorService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, Sensor* sens);                    
        void SensorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Sensor* sens);
        void FLSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                        stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, FLS* fls);
        void SSSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                        stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, SSS* sss);
        void MSISService(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req,
                        stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res, MSIS* msis);
        
        void ThrustersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot);
        void PropellersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot);
        void RuddersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot);
        void RotatingElementsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot);
        void ControlSurfacesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot);
        void ServosCallback(const sensor_msgs::msg::JointState::SharedPtr msg, std::shared_ptr<ROS2Robot> robot);
        
        void JointCallback(const std_msgs::msg::Float64::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::string& jointName);
        void JointGroupCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::vector<std::string>& jointNames);
        void GlueService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, FixedJoint* fix);
        void LightService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, Light* light);
        void CommCallback(const std_msgs::msg::String::SharedPtr msg, Comm* comm);
        
        // Sim speed up shit
        // mutable — accessed from const getSimulationClock()
        mutable std::mutex    clockMutex_;
        mutable uint64_t accumulatedSimUs_ = 1;   // was 0

        // plain — only touched by non-const SimulationClockSleep
        std::chrono::steady_clock::time_point nextWallDeadline_;
        bool   deadlineInitialized_ = false;
        double lastKnownRTF_         = 0.0;
        double customRTF;
        
        std::map<int64_t, AuthorityNode> authorityNodes_;
        std::atomic<bool> authorityVotes;
        uint64_t nominal_dt_us_ = 1000; 

    protected:
        std::string scenarioPath_;
        std::string dataPath_;
        std::vector<std::shared_ptr<ROS2Robot>> rosRobots_;
        std::shared_ptr<rclcpp::Node> nh_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
        std::shared_ptr<ROS2Interface> interface_;
        std::map<std::string, rclcpp::ServiceBase::SharedPtr> srvs_;
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> pubs_;
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
        std::map<std::string, image_transport::Publisher> imgPubs_;
        std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, 
            sensor_msgs::msg::CameraInfo::SharedPtr>> cameraMsgPrototypes_;
        std::map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr,
            sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>> dualImageCameraMsgPrototypes_;
        std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, 
            sensor_msgs::msg::Image::SharedPtr>> sonarMsgPrototypes_;
        
        // Sim speed up shit
        rclcpp::Time currentSimTime_;
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr waveHeights_pub_;
        rclcpp::Subscription<stonefish_ros2::msg::AuthorityNodeVote>::SharedPtr authorityVote_;
        rclcpp::Subscription<stonefish_ros2::msg::AuthorityNodeStep>::SharedPtr authorityStep_;
    
    private:
        std::vector<std::string> deletion_queue;
        std::mutex authority_mutex;
        std::condition_variable authority_cv;
        int64_t next_authority_id_ = 1;

        struct RobotSpawnRequest {
        std::shared_ptr<tinyxml2::XMLDocument> doc;   // keeps the parsed XML alive until the step consumes it
        tinyxml2::XMLElement* robotElem;
        std::string name;
        std::shared_ptr<std::promise<std::pair<bool,std::string>>> result;
        };
        struct RobotDeleteRequest {
            std::string name;
            std::shared_ptr<std::promise<std::pair<bool,std::string>>> result;
        };
        std::mutex spawnQueueMutex_;
        std::vector<RobotSpawnRequest> robotSpawnQueue_;
        std::mutex deleteQueueMutex_;
        std::vector<RobotDeleteRequest> robotDeleteQueue_;
        void DrainRobotSpawnQueue();
        void DrainRobotDeleteQueue();
    };
}

#endif