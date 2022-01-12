/*
 * Copyright 2016 Open Source Robotics Foundation
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

#ifndef _ROS_CONVEYOR_BELT_PLUGIN_HH_
#define _ROS_CONVEYOR_BELT_PLUGIN_HH_

#include <sdf/sdf.hh>

// Gazebo
#include "ConveyorBeltPlugin.hh"

// ROS
#include <gazebo_ros/node.hpp>
#include "gazebo_conveyor/srv/conveyor_belt_control.hpp"
#include "gazebo_conveyor/msg/conveyor_belt_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

namespace gazebo
{
  /// \brief ROS implementation of the ConveyorBeltPlugin plugin
  class ROSConveyorBeltPlugin : public ConveyorBeltPlugin
  {
    /// \brief Constructor
    public: ROSConveyorBeltPlugin();

    /// \brief Destructor
    public: virtual ~ROSConveyorBeltPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Receives requests on the conveyor belt's topic.
    /// \param[in] _req The desired state of the conveyor belt.
    /// \param[in] _res If the service succeeded or not.
    public: void OnControlCommand(std::shared_ptr<gazebo_conveyor::srv::ConveyorBeltControl::Request> request, std::shared_ptr<gazebo_conveyor::srv::ConveyorBeltControl::Response> response);

    public: void SubscriberCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    // Documentation inherited.
    private: virtual void Publish() const;

    /// \brief for setting ROS name space
    private: std::string robotNamespace_;

    /// \brief ros node handle
    //private: std::shared_ptr<rclcpp::Node> rosnode_;
    private: gazebo_ros::Node::SharedPtr rosnode_;

    /// \brief Receives service calls to control the conveyor belt.
    public: rclcpp::Service<gazebo_conveyor::srv::ConveyorBeltControl>::SharedPtr controlService_;
    public: rclcpp::Service<std_srvs::srv::Empty>::SharedPtr MySubscriber;

    /// \brief Publishes the state of the conveyor.
    public: rclcpp::Publisher<gazebo_conveyor::msg::ConveyorBeltState>::SharedPtr statePub;
  };
}
#endif
