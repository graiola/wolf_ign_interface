/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <wolf_ign_interface/wolf_ign_hw.h>

// ignition
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/msgs/imu.pb.h>

namespace wolf_ign_interface
{

using namespace hardware_interface;
using namespace ign_ros_control;


WolfRobotHwIgn::WolfRobotHwIgn()
  :IgnitionSystem()
{
}

WolfRobotHwIgn::~WolfRobotHwIgn() = default;

bool WolfRobotHwIgn::initSim(ros::NodeHandle model_nh,
                             std::map<std::string, ignition::gazebo::Entity> & enableJoints,
                             ignition::gazebo::EntityComponentManager & ecm,
                             std::vector<transmission_interface::TransmissionInfo> transmissions,
                             int & update_rate)
{
  ecm_ = &ecm;

  bool res = IgnitionSystem::initSim(model_nh,enableJoints,ecm,transmissions,update_rate);

  try
  {
    std::string selected_imu = WolfRobotHwInterface::loadImuLinkNameFromSRDF();
    if(ecm_->HasComponentType(ignition::gazebo::components::Imu().TypeId()))
    {
      ecm_->Each<ignition::gazebo::components::Imu,ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity & _entity,
            const ignition::gazebo::components::Imu * _imu,
            const ignition::gazebo::components::Name * _name) -> bool
      {
        if(selected_imu == _name->Data())
        {
          ROS_INFO_STREAM_NAMED(CLASS_NAME,"Loading IMU sensor: " << _name->Data());
          WolfRobotHwInterface::initializeImuInterface(_name->Data());

          sim_imu_sensor_ = _entity;

          auto sensorTopicComp = ecm_->Component<ignition::gazebo::components::SensorTopic>(_entity);
          if (sensorTopicComp)
          {
            ROS_INFO_STREAM_NAMED(CLASS_NAME,"Topic name: " << sensorTopicComp->Data());
            imu_topic_name_ = sensorTopicComp->Data();
          }
        }
        return true;
      });
    }
    else
      throw std::runtime_error("There is no imu sensor in the sdf file!");
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR_STREAM_NAMED(CLASS_NAME,"Can not load imu interface: " <<e.what());
    res = false;
  }
  return res;
}

void WolfRobotHwIgn::read()
{
  IgnitionSystem::read();
  if (imu_topic_name_.empty()) {
    auto sensorTopicComp = ecm_->Component<
        ignition::gazebo::components::SensorTopic>(sim_imu_sensor_);
    if (sensorTopicComp) {
      imu_topic_name_ = sensorTopicComp->Data();
      ROS_INFO_STREAM_NAMED(CLASS_NAME,"IMU sensor has a topic named " << sensorTopicComp->Data() << ", subscribing to it.");
      node_.Subscribe(imu_topic_name_,&WolfRobotHwIgn::readImu,this);
    }
  }
}

void WolfRobotHwIgn::write()
{
  IgnitionSystem::write();
}

void WolfRobotHwIgn::readImu(const ignition::msgs::IMU& msg)
{
  this->imu_orientation_[0] = msg.orientation().w();
  this->imu_orientation_[1] = msg.orientation().x();
  this->imu_orientation_[2] = msg.orientation().y();
  this->imu_orientation_[3] = msg.orientation().z();

  this->imu_ang_vel_[0] = msg.angular_velocity().x();
  this->imu_ang_vel_[1] = msg.angular_velocity().y();
  this->imu_ang_vel_[2] = msg.angular_velocity().z();

  this->imu_lin_acc_[0] = msg.linear_acceleration().x();
  this->imu_lin_acc_[1] = msg.linear_acceleration().y();
  this->imu_lin_acc_[2] = msg.linear_acceleration().z();
}

} // namespace

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
    wolf_ign_interface::WolfRobotHwIgn, ign_ros_control::IgnitionSystemInterface)
