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
#include "ROSConveyorBeltPlugin.hh"
#include "gazebo_conveyor/msg/conveyor_belt_state.hpp"
#include <gazebo_ros/node.hpp>

#include <cstdlib>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin);

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{

}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::cout << "in Load Function\n";
  // load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    if(_sdf->GetElement("robot_namespace")->Get<std::string>() != "/") {
          this->robotNamespace_ = _sdf->GetElement(
        "robot_namespace")->Get<std::string>() + "/";
    }

  }
  std::cout << "Got Namespace: " << this->robotNamespace_ << "\n";

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::is_initialized())
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ROSConveyorBeltPlugin"), "A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::cout << "Is initialized\n";

  std::string controlTopic = "conveyor/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "conveyor/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  std::cout << "Before Loading\n";
  ConveyorBeltPlugin::Load(_parent, _sdf);
  std::cout << "After Loading\n";
  //this->rosnode_ = rclcpp::Node::make_shared(this->robotNamespace_+"ROSConveyorBeltPlugin");
  this->rosnode_ = gazebo_ros::Node::Get(_sdf);
  std::cout << "After ROS Node\n";
  this->controlService_ = this->rosnode_->create_service<gazebo_conveyor::srv::ConveyorBeltControl>(controlTopic,
    std::bind(&ROSConveyorBeltPlugin::OnControlCommand, this, std::placeholders::_1, std::placeholders::_2));
  this->MySubscriber = this->rosnode_->create_service<std_srvs::srv::Empty>(
            "my_empty_service_name",
            std::bind(
                &ROSConveyorBeltPlugin::SubscriberCallback, 
                this, 
                std::placeholders::_1,              // Corresponds to the 'request' input
                std::placeholders::_2)
        );
  std::cout << "After Service Creation\n";
  // Message used for publishing the state of the conveyor.
  this->statePub = this->rosnode_->create_publisher<
    gazebo_conveyor::msg::ConveyorBeltState>(stateTopic, 1000);
  
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Publish() const
{
  gazebo_conveyor::msg::ConveyorBeltState stateMsg;
  stateMsg.enabled = this->IsEnabled();
  stateMsg.power = this->Power();
  this->statePub->publish(stateMsg);
}

void ROSConveyorBeltPlugin::SubscriberCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response>      response) {
          gzdbg << "COOL\n";
        }
/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::OnControlCommand(const std::shared_ptr<gazebo_conveyor::srv::ConveyorBeltControl::Request> request, std::shared_ptr<gazebo_conveyor::srv::ConveyorBeltControl::Response> response)
{

  gzdbg << "In Callback\n";
  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ROSConveyorBeltPlugin"), errStr);
    response->success = false;
  }

  if (!this->IsEnabled())
  {
    std::string errStr = "Belt is not currently enabled so power cannot be set. It may be congested.";
    gzerr << errStr << std::endl;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ROSConveyorBeltPlugin"), errStr);
    response->success = false;
  }

  if (!(0 == request->power || request->power <= 100))
  {
    std::string errStr = "Requested belt power is invalid. Accepted values are in the range [0, 100].";
    gzerr << errStr << std::endl;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ROSConveyorBeltPlugin"), errStr);
    response->success = false;
  }

  this->SetPower(request->power);
  response->success = true;
}
