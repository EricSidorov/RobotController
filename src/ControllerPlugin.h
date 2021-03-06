/*
 * Copyright 2013 SMILE Lab, Technion
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef CONTROLLER_PLUGIN_HH
#define CONTROLLER_PLUGIN_HH

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <RobotController/ResetControls.h>
#include <RobotController/ResetIC.h>
#include <RobotController/ControllerStatistics.h>

// don't use these to control
//#include <sensor_msgs/JointState.h>
//#include <osrf_msgs/JointCommands.h>

// high speed control
#include <RobotController/RobotState.h>
#include <RobotController/RobotCommand.h>

#include "PubQueue.h"

class MyContactSensor // : gazebo::sensors::ContactSensor
{
    /// \brief Constructor
    public: MyContactSensor();

    /// \brief Destructor
    public: virtual ~MyContactSensor();

    /// \brief connected by ContactUpdateConnection, called when contact
    /// sensor update
    public: void OnContactUpdate();

    public: std::string Name;
    public: gazebo::sensors::ContactSensorPtr SensorPtr;
    // public: boost::shared_ptr<gazebo::sensors::ContactSensorPtr> SensorPtr;
    public: gazebo::event::ConnectionPtr ContactUpdateConnection;
    public: ros::Publisher pubContact;
    public: PubQueue<std_msgs::Int32>::Ptr pubContactQueue;

    //private: int LastNumConnections;
    private: int state; // 1- contact, 0 -no contact
    private: int count;

};

namespace gazebo
{

  class ControllerPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ControllerPlugin();

    /// \brief Destructor
    public: virtual ~ControllerPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief get data from IMU for robot state
    private: void GetIMUState(const common::Time &_curTime);

    /// \brief get data from force torque sensor
    private: void GetForceTorqueSensorState(const common::Time &_curTime);

    /// \brief ros service callback to reset joint control internal states
    /// \param[in] _req Incoming ros service request
    /// \param[in] _res Outgoing ros service response
    private: bool ResetControls(RobotController::ResetControls::Request &_req,
      RobotController::ResetControls::Response &_res);

    private: bool ResetToIC(RobotController::ResetIC::Request &_req,
      RobotController::ResetIC::Response &_res);

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    /// Throttle update rate
    private: common::Time lastControllerStatisticsTime;
    private: double statsUpdateRate;

    // Contact sensors
    private: std::vector<MyContactSensor*> ContactSensors;

    // Force torque sensors at ankles
    private: std::vector<physics::JointPtr> AnkleJoints;

    // Force torque sensors at wrists
    private: std::vector<physics::JointPtr> WristJoints;

    /// \brief A combined RobotState, IMU and ForceTorqueSensors Message
    /// for accessing all these states synchronously.
    private: RobotController::RobotState RobotState;

    // IMU sensor
    private: boost::shared_ptr<sensors::ImuSensor> imuSensor;
    private: std::string imuLinkName;
    
    // deferred loading in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS internal stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;

    /// \brief ros publisher for ros controller timing statistics
    private: ros::Publisher pubControllerStatistics;
    private: PubQueue<RobotController::ControllerStatistics>::Ptr
      pubControllerStatisticsQueue;

    /// \brief ros publisher for force atlas joint states
    // private: ros::Publisher pubJointStates;
    // private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

    /// \brief ros publisher for robot states, currently it contains
    /// joint index enums
    /// sensor_msgs::JointState
    /// sensor_msgs::Imu
    /// RobotController::FroceTorqueSensors
    private: ros::Publisher pubRobotState;
    private: PubQueue<RobotController::RobotState>::Ptr pubRobotStateQueue;

    private: ros::Subscriber subRobotCommand;

    /// \brief ros topic callback to update Joint Commands
    /// \param[in] _msg Incoming ros message
    private: void SetCommand(
      const RobotController::RobotCommand::ConstPtr &_msg);

    private: void Pause(const std_msgs::String::ConstPtr &_msg);
    private: boost::condition pause;
    private: ros::Subscriber subPause;
    private: boost::mutex pauseMutex;

    private: void LoadPIDGainsFromParameter();
    private: void ZeroCommand();

    private: std::vector<std::string> jointNames;

    // JointController: pointer to a copy of the joint controller in gazebo
    private: physics::JointControllerPtr jointController;
    private: transport::NodePtr node;
    private: transport::PublisherPtr jointCmdPub;

    /// \brief Internal list of pointers to Joints
    private: physics::Joint_V joints;
    private: std::vector<double> effortLimit;

    /// \brief internal class for keeping track of PID states
    private: class ErrorTerms
      {
        /// error term contributions to final control output
        double q_p;
        double d_q_p_dt;
        double k_i_q_i;  // integral term weighted by k_i
        double qd_p;
        friend class ControllerPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: RobotController::RobotCommand RobotCommand;
    private: boost::mutex mutex;

    /// \brief ros service to reset controls internal states
    private: ros::ServiceServer resetControlsService;
    private: ros::ServiceServer resetToICService;

    /// \brief Conversion functions
    // private: inline math::Pose ToPose(const geometry_msgs::Pose &_pose) const
    // {
    //   return math::Pose(math::Vector3(_pose.position.x,
    //                                   _pose.position.y,
    //                                   _pose.position.z),
    //                     math::Quaternion(_pose.orientation.w,
    //                                      _pose.orientation.x,
    //                                      _pose.orientation.y,
    //                                      _pose.orientation.z));
    // }

    // /// \brief Conversion helper functions
    // private: inline geometry_msgs::Pose ToPose(const math::Pose &_pose) const
    // {
    //   geometry_msgs::Pose result;
    //   result.position.x = _pose.pos.x;
    //   result.position.y = _pose.pos.y;
    //   result.position.z = _pose.pos.y;
    //   result.orientation.w = _pose.rot.w;
    //   result.orientation.x = _pose.rot.x;
    //   result.orientation.y = _pose.rot.y;
    //   result.orientation.z = _pose.rot.z;
    //   return result;
    // }

    // /// \brief Conversion helper functions
    // private: inline geometry_msgs::Point ToPoint(const AtlasVec3f &_v) const
    // {
    //   geometry_msgs::Point result;
    //   result.x = _v.n[0];
    //   result.y = _v.n[1];
    //   result.z = _v.n[2];
    //   return result;
    // }

    // /// \brief Conversion helper functions
    // private: inline geometry_msgs::Quaternion ToQ(const math::Quaternion &_q)
    //   const
    // {
    //   geometry_msgs::Quaternion result;
    //   result.w = _q.w;
    //   result.x = _q.x;
    //   result.y = _q.y;
    //   result.z = _q.z;
    //   return result;
    // }

    // /// \brief Conversion helper functions
    // private: inline AtlasVec3f ToVec3(const geometry_msgs::Point &_point) const
    // {
    //   return AtlasVec3f(_point.x,
    //                     _point.y,
    //                     _point.z);
    // }

    // /// \brief Conversion helper functions
    // private: inline AtlasVec3f ToVec3(const math::Vector3 &_vector3) const
    // {
    //   return AtlasVec3f(_vector3.x,
    //                     _vector3.y,
    //                     _vector3.z);
    // }

    // /// \brief Conversion helper functions
    // private: inline math::Vector3 ToVec3(const AtlasVec3f &_vec3) const
    // {
    //   return math::Vector3(_vec3.n[0],
    //                        _vec3.n[1],
    //                        _vec3.n[2]);
    // }

    // /// \brief Conversion helper functions
    // private: inline geometry_msgs::Vector3 ToGeomVec3(
    //   const AtlasVec3f &_vec3) const
    // {
    //   geometry_msgs::Vector3 result;
    //   result.x = _vec3.n[0];
    //   result.y = _vec3.n[1];
    //   result.z = _vec3.n[2];
    //   return result;
    // }

    /// \brief: for keeping track of internal controller update rates.
    private: common::Time lastControllerUpdateTime;

    // controls message age measure
    private: RobotController::ControllerStatistics controllerStatistics;
    private: std::vector<double> RobotCommandAgeBuffer;
    private: std::vector<double> RobotCommandAgeDelta2Buffer;
    private: unsigned int RobotCommandAgeBufferIndex;
    private: double RobotCommandAgeBufferDuration;
    private: double RobotCommandAgeMean;
    private: double RobotCommandAgeVariance;
    private: double RobotCommandAge;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
  };

}
#endif
