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

#ifndef WOLF_IGN_HW_H
#define WOLF_IGN_HW_H

// ROS includes
#include <ros/ros.h>
#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>

// ign_ros_control
#include <ign_ros_control/ign_system_interface.hpp>
#include <ign_ros_control/ign_system.hpp>

// URDF include
#include <urdf/model.h>

// HW interface
#include <wolf_hardware_interface/wolf_robot_hw.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// ignition
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/components/Imu.hh>

namespace wolf_ign_interface
{

/**
 * @class This class implements the Ignition hardware interface for a generic robot to control through a whole body controller
 * @brief This hardware interface is loaded to Ignition using ign_ros_control plugin
 * which required the initSim, read and write methods override in this class.
 */
class WolfRobotHwIgn : public ign_ros_control::IgnitionSystem, public hardware_interface::WolfRobotHwInterface
{
public:

  typedef std::shared_ptr<WolfRobotHwIgn> Ptr;

  const std::string CLASS_NAME = "WolfRobotHwIgn";

  WolfRobotHwIgn();
  virtual ~WolfRobotHwIgn();

  // Documentation Inherited
  bool initSim(
      ros::NodeHandle model_nh,
      std::map<std::string, ignition::gazebo::Entity> & joints,
      ignition::gazebo::EntityComponentManager & ecm,
      std::vector<transmission_interface::TransmissionInfo> transmissions,
      int & update_rate) override;

  //// Documentation Inherited
  virtual void read() override;
  //
  //// Documentation Inherited
  virtual void write() override;

  void readImu(const ignition::msgs::IMU& msg);

private:

  std::string imu_topic_name_;

  ignition::gazebo::EntityComponentManager* ecm_;

  const ignition::gazebo::components::Imu* sim_imu_sensor_;

  ignition::transport::Node node_;

  ignition::msgs::IMU imu_msg_;

};

}

#endif
