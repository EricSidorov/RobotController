/*
 * Copyright 2013 SMILE Lab
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

#include <string>
#include <algorithm>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "ControllerPlugin.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)

////////////////////////////////////////////////////////////////////////////////
ControllerPlugin::ControllerPlugin()
{
  // the parent link of the imu_sensor ends up being pelvis after
  // fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_senosr block.
  this->imuLinkName = "imu_link";
}

////////////////////////////////////////////////////////////////////////////////
ControllerPlugin::~ControllerPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();

  // JointController: built-in gazebo to control joints
  this->jointController = this->model->GetJointController();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  this->jointCmdPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + this->model->GetName() + "/joint_cmd");

  // save sdf
  this->sdf = _sdf;

  // initialize update time
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // init joints, automated from model
  physics::Joint_V Joints = this->model->GetJoints();
  for (physics::Joint_V::iterator it = Joints.begin(); it != Joints.end(); ++it) {
    this->jointNames.push_back((*it)->GetName());
    std::cout << "Added joint: " << this->jointNames.back() << "\n";
  }
  // instead of hardcoded by:
  // this->jointNames.push_back("back_lbz");
  
  // get pointers to joints from gazebo
  this->joints.resize(this->jointNames.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    if (!this->joints[i])
    {
      ROS_ERROR("robot expected joint '%s' not present, plugin not loaded",
        this->jointNames[i].c_str());
      return;
    }
  }

  // get effort limits from gazebo
  this->effortLimit.resize(this->jointNames.size());
  for (unsigned i = 0; i < this->effortLimit.size(); ++i)
    this->effortLimit[i] = this->joints[i]->GetEffortLimit(0);

  // JointController: Publish messages to reset joint controller gains
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    msgs::JointCmd msg;
    msg.set_name(this->joints[i]->GetScopedName());
    msg.mutable_position()->set_target(0.0);
    msg.mutable_position()->set_p_gain(0.0);
    msg.mutable_position()->set_i_gain(0.0);
    msg.mutable_position()->set_d_gain(0.0);
    msg.mutable_position()->set_i_max(0.0);
    msg.mutable_position()->set_i_min(0.0);
    msg.mutable_position()->set_limit(0.0);
  }

  {
    // initialize PID states: error terms
    this->errorTerms.resize(this->joints.size());
    for (unsigned i = 0; i < this->errorTerms.size(); ++i)
    {
      this->errorTerms[i].q_p = 0;
      this->errorTerms[i].d_q_p_dt = 0;
      this->errorTerms[i].k_i_q_i = 0;
      this->errorTerms[i].qd_p = 0;
    }
  }

  {
    this->RobotState.position.resize(this->joints.size());
    this->RobotState.velocity.resize(this->joints.size());
    this->RobotState.effort.resize(this->joints.size());
    this->RobotState.kp_position.resize(this->joints.size());
    this->RobotState.ki_position.resize(this->joints.size());
    this->RobotState.kd_position.resize(this->joints.size());
    this->RobotState.kp_velocity.resize(this->joints.size());
    this->RobotState.i_effort_min.resize(this->joints.size());
    this->RobotState.i_effort_max.resize(this->joints.size());
  }

  {
    this->RobotCommand.position.resize(this->joints.size());
    this->RobotCommand.velocity.resize(this->joints.size());
    this->RobotCommand.effort.resize(this->joints.size());
    this->RobotCommand.kp_position.resize(this->joints.size());
    this->RobotCommand.ki_position.resize(this->joints.size());
    this->RobotCommand.kd_position.resize(this->joints.size());
    this->RobotCommand.kp_velocity.resize(this->joints.size());
    this->RobotCommand.i_effort_min.resize(this->joints.size());
    this->RobotCommand.i_effort_max.resize(this->joints.size());

    this->ZeroCommand();
  }

  // Get joints that have force/torque sensors by name "ankle" or "wrist"
  std::string str1 ("ankle");
  std::string str2 ("wrist");
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    if (this->jointNames[i].find(str1) != std::string::npos) {
      this->AnkleJoints.push_back(this->model->GetJoint(this->jointNames[i]));
      std::cout << "Added ankle force/torque sensor at joint: " << this->AnkleJoints.back()->GetName() << "\n";
    }
    if (this->jointNames[i].find(str2) != std::string::npos) {
      this->WristJoints.push_back(this->model->GetJoint(this->jointNames[i]));
      std::cout << "Added wrist force/torque sensor at joint: " << this->AnkleJoints.back()->GetName() << "\n";
    }
  }

  this->RobotState.ankles.resize(this->AnkleJoints.size());
  this->RobotState.wrists.resize(this->WristJoints.size());

  // Get sensors
  //this->imuSensor =
  //  boost::shared_dynamic_cast<sensors::ImuSensor>
  //    (sensors::SensorManager::Instance()->GetSensor(
  //      this->world->GetName() + "::" + this->model->GetScopedName()
  //      + "::inner_legs::"
  //      "imu_sensor"));
  
  sensors::Sensor_V Sensors = sensors::SensorManager::Instance()->GetSensors();
  std::string imustr ("imu_sensor");
  std::string contstr ("contact_sensor");
  for (sensors::Sensor_V::iterator it = Sensors.begin(); it != Sensors.end(); ++it) {
    if ( (*it)->GetName().find(imustr) != std::string::npos ) {
      this->imuSensor = boost::shared_dynamic_cast<sensors::ImuSensor>(*it);
    }
    if ( (*it)->GetName().find(contstr) != std::string::npos ) {
      this->ContactSensors.push_back(new MyContactSensor());
      this->ContactSensors.back()->Name = (*it)->GetName();
      this->ContactSensors.back()->SensorPtr =
        boost::shared_dynamic_cast<sensors::ContactSensor>(*it);
      std::cout << "Added contact sensor " << (*it)->GetName() << "\n" << "\n";
    }
  }
  if (!this->imuSensor)
    gzerr << "imu_sensor not found\n" << "\n";
  if (this->ContactSensors.empty())
    std::cout << "no contact sensor found\n" << "\n";

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&ControllerPlugin::DeferredLoad, this));
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::Pause(
  const std_msgs::String::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->pauseMutex);
  this->pause.notify_one();
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::SetCommand(
  const RobotController::RobotCommand::ConstPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->RobotCommand.header.stamp = _msg->header.stamp;

  // for RobotCommand, only position, velocity and efforts are used.
  if (_msg->position.size() == this->RobotCommand.position.size())
    std::copy(_msg->position.begin(), _msg->position.end(),
      this->RobotCommand.position.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg->position.size(), this->RobotCommand.position.size());

  if (_msg->velocity.size() == this->RobotCommand.velocity.size())
    std::copy(_msg->velocity.begin(), _msg->velocity.end(),
      this->RobotCommand.velocity.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg->velocity.size(), this->RobotCommand.velocity.size());

  if (_msg->effort.size() == this->RobotCommand.effort.size())
    std::copy(_msg->effort.begin(), _msg->effort.end(),
      this->RobotCommand.effort.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg->effort.size(), this->RobotCommand.effort.size());

  // the rest are stored in RobotState for publication
  if (_msg->kp_position.size() == this->RobotState.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
      this->RobotState.kp_position.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->RobotState.kp_position.size());

  if (_msg->ki_position.size() == this->RobotState.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
      this->RobotState.ki_position.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->RobotState.ki_position.size());

  if (_msg->kd_position.size() == this->RobotState.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
      this->RobotState.kd_position.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->RobotState.kd_position.size());

  if (_msg->kp_velocity.size() == this->RobotState.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
      this->RobotState.kp_velocity.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->RobotState.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->RobotState.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
      this->RobotState.i_effort_min.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->RobotState.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->RobotState.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
      this->RobotState.i_effort_max.begin());
  else
    ROS_DEBUG("RobotCommand message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->RobotState.i_effort_max.size());
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // publish multi queue
  this->pmq.startServiceThread();

  // pull down controller parameters
  this->LoadPIDGainsFromParameter();

  // Get window size from ros parameter server (seconds)
  if (!this->rosNode->getParam(
    this->model->GetName() + "/statistics_time_window_size",
    this->RobotCommandAgeBufferDuration))
  {
    this->RobotCommandAgeBufferDuration = 1.0;
    ROS_INFO("controller statistics window size not specified in"
             " ros parameter server, defaulting to %f sec.",
             this->RobotCommandAgeBufferDuration);
  }

  double stepSize = this->world->GetPhysicsEngine()->GetMaxStepSize();
  if (math::equal(stepSize, 0.0))
  {
    stepSize = 0.001;
    ROS_WARN("simulation step size is zero, something is wrong,"
              "  Defaulting to step size of %f sec.", stepSize);
  }

  // document this from
  // http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  // Online algorithm
  // where Delta2 buffer contains delta*(x - mean) line from code block
  unsigned int bufferSize = this->RobotCommandAgeBufferDuration / stepSize;
  this->RobotCommandAgeBuffer.resize(bufferSize);
  this->RobotCommandAgeDelta2Buffer.resize(bufferSize);
  this->RobotCommandAgeBufferIndex = 0;
  this->RobotCommandAgeMean = 0.0;
  this->RobotCommandAgeVariance = 0.0;

  this->pubRobotStateQueue = this->pmq.addPub<RobotController::RobotState>();
  this->pubRobotState = this->rosNode->advertise<RobotController::RobotState>(
    this->model->GetName() + "/robot_state", 1);

  // ros publication / subscription
  this->pubControllerStatisticsQueue =
    this->pmq.addPub<RobotController::ControllerStatistics>();
  this->pubControllerStatistics =
    this->rosNode->advertise<RobotController::ControllerStatistics>(
    this->model->GetName() + "/controller_statistics", 10);

  for (std::vector<MyContactSensor*>::iterator it = this->ContactSensors.begin(); it != this->ContactSensors.end(); ++it) {
    (*it)->pubContact =
      this->rosNode->advertise<std_msgs::Int32>(
        this->model->GetName()+"/"+(*it)->Name, 10);
    (*it)->pubContactQueue = this->pmq.addPub<std_msgs::Int32>();

    // on contact
    (*it)->ContactUpdateConnection = (*it)->SensorPtr->ConnectUpdated(
       boost::bind(&MyContactSensor::OnContactUpdate, (*it)));

  }

  // ros topic subscribtions
  ros::SubscribeOptions pauseSo =
    ros::SubscribeOptions::create<std_msgs::String>(
    this->model->GetName() + "/pause", 1,
    boost::bind(&ControllerPlugin::Pause, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  pauseSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);
  this->subPause =
    this->rosNode->subscribe(pauseSo);

  // ros topic subscribtions
  ros::SubscribeOptions RobotCommandSo =
    ros::SubscribeOptions::create<RobotController::RobotCommand>(
    this->model->GetName() + "/robot_command", 1,
    boost::bind(&ControllerPlugin::SetCommand, this, _1),
    ros::VoidPtr(), &this->rosQueue);

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint commands, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  RobotCommandSo.transport_hints =
    ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

  this->subRobotCommand =
    this->rosNode->subscribe(RobotCommandSo);

  // initialize status pub time
  this->lastControllerStatisticsTime = this->world->GetSimTime().Double();

  // controller statistics update rate defaults to 1kHz,
  // read from ros param if available
  double rate;
  if (this->rosNode->getParam(this->model->GetName() + "/controller_statistics/update_rate",
    rate))
  {
    rate = math::clamp(rate, 1.0, 10000.0);
    ROS_INFO("ControllerPlugin controller statistics %f kHz", rate);
    this->statsUpdateRate = rate;
  }
  else
  {
    ROS_INFO("ControllerPlugin default controller statistics 1kHz");
    this->statsUpdateRate = 1000.0;
  }

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&ControllerPlugin::RosQueueThread, this));


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&ControllerPlugin::UpdateStates, this));

  
  // Advertise services on the custom queue
  ros::AdvertiseServiceOptions resetControlsAso =
    ros::AdvertiseServiceOptions::create<RobotController::ResetControls>(
      this->model->GetName() + "/reset_controls", boost::bind(
        &ControllerPlugin::ResetControls, this, _1, _2),
        ros::VoidPtr(), &this->rosQueue);

    ros::AdvertiseServiceOptions resetToICAso =
    ros::AdvertiseServiceOptions::create<RobotController::ResetIC>(
      this->model->GetName() + "/reset_IC", boost::bind(
        &ControllerPlugin::ResetToIC, this, _1, _2),
        ros::VoidPtr(), &this->rosQueue);

  
  this->resetControlsService = this->rosNode->advertiseService(
    resetControlsAso);

  this->resetToICService = this->rosNode->advertiseService(
    resetToICAso);
  
}

////////////////////////////////////////////////////////////////////////////////
bool ControllerPlugin::ResetControls(RobotController::ResetControls::Request &_req,
  RobotController::ResetControls::Response &_res)
{
  boost::mutex::scoped_lock lock(this->mutex);

  _res.success = true;
  _res.status_message = "success";

  if (_req.reset_pid_controller)
    for (unsigned i = 0; i < this->errorTerms.size(); ++i)
    {
      this->errorTerms[i].q_p = 0;
      this->errorTerms[i].d_q_p_dt = 0;
      this->errorTerms[i].k_i_q_i = 0;
      this->errorTerms[i].qd_p = 0;
    }

  if (_req.reload_pid_from_ros)
    this->LoadPIDGainsFromParameter();
  else
  {
    // boost::shared_ptr<RobotController::RobotCommand> msg(_req.robot_command);
    this->SetCommand(
      static_cast<RobotController::RobotCommand::ConstPtr>(&(_req.robot_command)));
  }

  return _res.success;
}

////////////////////////////////////////////////////////////////////////////////
bool ControllerPlugin::ResetToIC(RobotController::ResetIC::Request &_req,
  RobotController::ResetIC::Response &_res)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // physics::PhysicsEnginePtr Eng;
  // Eng = this->world->GetPhysicsEngine();
  this->model->Reset();
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    std::string JointName = this->model->GetName()+"::"+this->joints[i]->GetName();
    this->model->SetJointPosition(JointName,_req.joint_pos[i]);
    this->joints[i]->SetVelocity(0,_req.joint_vel[i]);
    this->joints[i]->Update();
  }
  math::Pose pose(_req.pose[0],_req.pose[1],_req.pose[2],_req.pose[3],_req.pose[4],_req.pose[5]);
  this->model->SetLinkWorldPose(pose, _req.reference_link);
  
  _res.success = true;
  _res.status_message = "success";
  
  return _res.success;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();

  
  double dt = (curTime - this->lastControllerUpdateTime).Double();

  if (curTime > this->lastControllerUpdateTime)
  {
    // get imu data from imu sensor
    this->GetIMUState(curTime);
    // get force torque sensor data from sensor
    this->GetForceTorqueSensorState(curTime);


    // populate RobotState from robot
    this->RobotState.header.stamp = ros::Time(curTime.sec, curTime.nsec);

    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->RobotState.position[i] = this->joints[i]->GetAngle(0).Radian();
      this->RobotState.velocity[i] = this->joints[i]->GetVelocity(0);
    }

    {
      boost::mutex::scoped_lock lock(this->mutex);
      {
        // Keep track of age of RobotCommand age in seconds.
        // Note the value is invalid as a moving window average age
        // until the buffer is full.
        this->RobotCommandAge = curTime.Double() -
          this->RobotCommand.header.stamp.toSec();

        double weightedRobotCommandAge = this->RobotCommandAge
          / this->RobotCommandAgeBuffer.size();

        // for variance calculation, save delta before average is updated.
        double delta = this->RobotCommandAge - this->RobotCommandAgeMean;

        // update average
        this->RobotCommandAgeMean += weightedRobotCommandAge;
        this->RobotCommandAgeMean -=
          this->RobotCommandAgeBuffer[this->RobotCommandAgeBufferIndex];

        // update variance with new average
        double delta2 = delta *
          (this->RobotCommandAge - this->RobotCommandAgeMean);
        this->RobotCommandAgeVariance += delta2;
        this->RobotCommandAgeVariance -=
          this->RobotCommandAgeDelta2Buffer[
          this->RobotCommandAgeBufferIndex];

        // save weighted average in window
        this->RobotCommandAgeBuffer[this->RobotCommandAgeBufferIndex] =
          weightedRobotCommandAge;

        // save delta buffer for incremental variance calculation
        this->RobotCommandAgeDelta2Buffer[
          this->RobotCommandAgeBufferIndex] = delta2;

        this->RobotCommandAgeBufferIndex =
         (this->RobotCommandAgeBufferIndex + 1) %
         this->RobotCommandAgeBuffer.size();
      }

      /// update pid with feedforward force
      for (unsigned int i = 0; i < this->joints.size(); ++i)
      {
        // truncate joint position within range of motion
        double positionTarget = math::clamp(
          this->RobotCommand.position[i],
          this->joints[i]->GetLowStop(0).Radian(),
          this->joints[i]->GetHighStop(0).Radian());

        double q_p = positionTarget - this->RobotState.position[i];

        if (!math::equal(dt, 0.0))
          this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

        this->errorTerms[i].q_p = q_p;

        this->errorTerms[i].qd_p =
          this->RobotCommand.velocity[i] - this->RobotState.velocity[i];

        this->errorTerms[i].k_i_q_i = math::clamp(
          this->errorTerms[i].k_i_q_i +
          dt * this->RobotState.ki_position[i] * this->errorTerms[i].q_p,
          static_cast<double>(this->RobotState.i_effort_min[i]),
          static_cast<double>(this->RobotState.i_effort_max[i]));

        // use gain params to compute force cmd
        double forceUnclamped =
          this->RobotState.kp_position[i] * this->errorTerms[i].q_p +
                                            this->errorTerms[i].k_i_q_i +
          this->RobotState.kd_position[i] * this->errorTerms[i].d_q_p_dt +
          this->RobotState.kp_velocity[i] * this->errorTerms[i].qd_p +
                                            this->RobotCommand.effort[i];

        // keep unclamped force for integral tie-back calculation
        double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i],
          this->effortLimit[i]);

        // integral tie-back during control saturation if using integral gain
        if (!math::equal(forceClamped,forceUnclamped) &&
            !math::equal((double)this->RobotState.ki_position[i],0.0) )
        {
          // lock integral term to provide continuous control as system moves
          // out of staturation
          this->errorTerms[i].k_i_q_i = math::clamp(
            this->errorTerms[i].k_i_q_i + (forceClamped - forceUnclamped),
          static_cast<double>(this->RobotState.i_effort_min[i]),
          static_cast<double>(this->RobotState.i_effort_max[i]));
        }

        // clamp force after integral tie-back
        forceClamped = math::clamp(forceUnclamped,
          -this->effortLimit[i], this->effortLimit[i]);

        // apply force to joint
        this->joints[i]->SetForce(0, forceClamped);

        // fill in jointState efforts
        this->RobotState.effort[i] = forceClamped;
      }
    }
    this->lastControllerUpdateTime = curTime;
    this->pubRobotStateQueue->push(this->RobotState, this->pubRobotState);
    /// controller statistics diagnostics, damages, etc.
    if (this->pubControllerStatistics.getNumSubscribers() > 0)
    {
      if ((curTime - this->lastControllerStatisticsTime).Double() >=
        1.0/this->statsUpdateRate)
      {
        RobotController::ControllerStatistics msg;
        msg.header.stamp = ros::Time(curTime.sec, curTime.nsec);
        msg.command_age = this->RobotCommandAge;
        msg.command_age_mean = this->RobotCommandAgeMean;
        msg.command_age_variance = this->RobotCommandAgeVariance /
          (this->RobotCommandAgeBuffer.size() - 1);
        msg.command_age_window_size = this->RobotCommandAgeBufferDuration;

        this->pubControllerStatisticsQueue->push(msg,
          this->pubControllerStatistics);
        this->lastControllerStatisticsTime = curTime;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::ZeroCommand()
{
  for (unsigned i = 0; i < this->jointNames.size(); ++i)
  {
    this->RobotCommand.position[i] = 0;
    this->RobotCommand.velocity[i] = 0;
    this->RobotCommand.effort[i] = 0;
    // store these directly on altasState, more efficient for pub later
    this->RobotState.kp_position[i] = 0;
    this->RobotState.ki_position[i] = 0;
    this->RobotState.kd_position[i] = 0;
    this->RobotState.kp_velocity[i] = 0;
    this->RobotState.i_effort_min[i] = 0;
    this->RobotState.i_effort_max[i] = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::LoadPIDGainsFromParameter()
{
  // pull down controller parameters
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    char joint_ns[200] = "";
    string gain_str = this->model->GetName() + "/gains/%s/";
    snprintf(joint_ns, sizeof(joint_ns), gain_str.c_str(),
             this->joints[i]->GetName().c_str());
    // this is so ugly
    double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
    string p_str = string(joint_ns)+"p";
    string i_str = string(joint_ns)+"i";
    string d_str = string(joint_ns)+"d";
    string i_clamp_str = string(joint_ns)+"i_clamp";
    if (!this->rosNode->getParam(p_str, p_val) ||
        !this->rosNode->getParam(i_str, i_val) ||
        !this->rosNode->getParam(d_str, d_val) ||
        !this->rosNode->getParam(i_clamp_str, i_clamp_val))
    {
      ROS_ERROR("couldn't find a param for %s", joint_ns);
      continue;
    }
    // store these directly on RobotState, more efficient for pub later
    this->RobotState.kp_position[i]  =  p_val;
    this->RobotState.ki_position[i]  =  i_val;
    this->RobotState.kd_position[i]  =  d_val;
    this->RobotState.i_effort_min[i] = -i_clamp_val;
    this->RobotState.i_effort_max[i] =  i_clamp_val;
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::GetIMUState(const common::Time &_curTime)
{
  if (this->imuSensor)
  {
    // compute angular rates
    {
      math::Vector3 wLocal = this->imuSensor->GetAngularVelocity();
      this->RobotState.angular_velocity.x = wLocal.x;
      this->RobotState.angular_velocity.y = wLocal.y;
      this->RobotState.angular_velocity.z = wLocal.z;
    }

    // compute acceleration
    {
      math::Vector3 accel = this->imuSensor->GetLinearAcceleration();
      this->RobotState.linear_acceleration.x = accel.x;
      this->RobotState.linear_acceleration.y = accel.y;
      this->RobotState.linear_acceleration.z = accel.z;
    }

    // compute orientation
    {
      math::Quaternion imuRot = this->imuSensor->GetOrientation();
      this->RobotState.orientation.x = imuRot.x;
      this->RobotState.orientation.y = imuRot.y;
      this->RobotState.orientation.z = imuRot.z;
      this->RobotState.orientation.w = imuRot.w;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::GetForceTorqueSensorState(const common::Time &_curTime)
{
  // get force torque at ankles and publish
  int ank=0;

  for (physics::Joint_V::iterator it = this->AnkleJoints.begin(); it != this->AnkleJoints.end(); ++it) {
    physics::JointWrench wrench = (*it)->GetForceTorque(0u);
    this->RobotState.ankles[ank].force.z = wrench.body2Force.z;
    this->RobotState.ankles[ank].torque.x = wrench.body2Torque.x;
    this->RobotState.ankles[ank].torque.y = wrench.body2Torque.y;
    ++ank;
  }
  // get force torque at wrists and publish
  int wri=0;
  for (physics::Joint_V::iterator it = this->WristJoints.begin(); it != this->WristJoints.end(); ++it) {
    physics::JointWrench wrench = (*it)->GetForceTorque(0u);
    this->RobotState.wrists[wri].force.z = wrench.body2Force.z;
    this->RobotState.wrists[wri].torque.x = wrench.body2Torque.x;
    this->RobotState.wrists[wri].torque.y = wrench.body2Torque.y;
    ++wri;
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

}

////////////////////////////////////////////////////////////////////////////////
MyContactSensor::MyContactSensor()
{

}


////////////////////////////////////////////////////////////////////////////////
MyContactSensor::~MyContactSensor()
{

}


////////////////////////////////////////////////////////////////////////////////
void MyContactSensor::OnContactUpdate()
{
  // Get all the contacts.
  gazebo::msgs::Contacts contacts;
  contacts = this->SensorPtr->GetContacts();
  std_msgs::Int32 msg;
  // std::cout << this->Name << "contact update " << contacts.contact_size()<<"\n";
  if (contacts.contact_size() == 0 && this->LastNumConnections!=0) {
    msg.data = 0;
    std::cout << "Disconnected" << "\n";
    this->pubContactQueue->push(msg, this->pubContact);
  }
  if (contacts.contact_size() > 0 && this->LastNumConnections==0) {
    msg.data = 1;
    std::cout << "Connected" << "\n";
    this->pubContactQueue->push(msg, this->pubContact);
  }

  this->LastNumConnections=contacts.contact_size();
}
