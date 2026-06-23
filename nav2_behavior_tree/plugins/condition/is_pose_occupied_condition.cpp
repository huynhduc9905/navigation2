// Copyright (c) 2025 Maurice Alexander Purnawan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_behavior_tree/plugins/condition/is_pose_occupied_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

IsPoseOccupiedCondition::IsPoseOccupiedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  use_footprint_(true), consider_unknown_as_obstacle_(false), cost_threshold_(254)
{
  still_stuck_ = false;
  use_stuck_signal_ = false;
  initialize();
}

void IsPoseOccupiedCondition::initialize()
{
  getInput<double>("cost_threshold", cost_threshold_);
  getInput<bool>("use_footprint", use_footprint_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  getInput("use_stuck_signal", use_stuck_signal_);
  getInputOrBlackboard("server_timeout", server_timeout_);
  createROSInterfaces();
}

void IsPoseOccupiedCondition::createROSInterfaces()
{
  std::string service_new;
  getInput<std::string>("service_name", service_new);
  if (service_new != service_name_ || !client_) {
    service_name_ = service_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ = std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::GetCosts>>(service_name_, node_, false /* Does not create and spin an internal executor*/);
      // node_->create_client<nav2_msgs::srv::GetCosts>(
      // service_name_,
      // false /* Does not create and spin an internal executor*/);
    rclcpp::QoS node_signal_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    node_status_pub_ = rclcpp::create_publisher<NodeSignal>(node_, "/node_signal", node_signal_qos);
    warning_cmd_pub_ = rclcpp::create_publisher<WarningCommand>(node_, "/audio/warn/command", 10);
  }
}

void IsPoseOccupiedCondition::sendSignals(bool isStuck, bool newPose)
{
  NodeSignal stuck_signal_msg_;
  stuck_signal_msg_.signal = NodeSignal::STUCK;
  stuck_signal_msg_.state = isStuck;
  node_status_pub_->publish(stuck_signal_msg_);

  if (newPose) {
    return;
  }

  WarningCommand warning_cmd_msg;
  if (isStuck) {
    warning_cmd_msg.cmd = WarningCommand::PLAY_SIGNAL;
    warning_cmd_msg.signal = WarningCommand::ROBOT_STUCK;
    warning_cmd_msg.repeat = true;
    warning_cmd_msg.period_s = 2;
    warning_cmd_pub_->publish(warning_cmd_msg);
  } else {
    warning_cmd_msg.cmd = WarningCommand::STOP;
    warning_cmd_pub_->publish(warning_cmd_msg);
  }
}

BT::NodeStatus IsPoseOccupiedCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
  geometry_msgs::msg::PoseStamped pose;
  getInput("pose", pose);

  bool same_pose =
    (current_pose_.header.frame_id == pose.header.frame_id) &&
    (current_pose_.header.stamp == pose.header.stamp) &&
    (current_pose_.pose == pose.pose);

  if (!same_pose) {
    // RCLCPP_INFO(node_->get_logger(), "DEBUG: Not The same goal");
    current_pose_ = pose;
    if (use_stuck_signal_) {
      sendSignals(false, true);
    }
    still_stuck_ = false;
  } 

  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  request->use_footprint = use_footprint_;
  request->poses.push_back(pose);

  auto response = client_->invoke(request);

  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetCosts service call failed");
    return BT::NodeStatus::FAILURE;
  }

  if ((response->costs[0] == 255 && !consider_unknown_as_obstacle_) ||
    response->costs[0] < cost_threshold_)
  {
    if (still_stuck_ && use_stuck_signal_) {
      sendSignals(false, false);
      still_stuck_ = false;
    }
    return BT::NodeStatus::FAILURE;
  } else {
    if (!still_stuck_ && use_stuck_signal_) {
      sendSignals(true, false);
      still_stuck_ = true;
      RCLCPP_INFO(node_->get_logger(), "NOTIFICATION-WARN: Obstacle on goal");
    }
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPoseOccupiedCondition>("IsPoseOccupied");
}