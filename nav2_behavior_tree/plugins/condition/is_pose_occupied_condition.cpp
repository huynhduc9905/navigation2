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
  use_footprint_(true), consider_unknown_as_obstacle_(false), cost_threshold_(254),
  service_name_("global_costmap/get_cost_global_costmap"),
  costmap_(nullptr)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  costmap_ros_ = config().blackboard->get<std::shared_ptr<nav2_costmap_2d::Costmap2DROS>>("costmap_ros");
}

void IsPoseOccupiedCondition::initialize()
{
  getInput<std::string>("service_name", service_name_);
  getInput<double>("cost_threshold", cost_threshold_);
  getInput<bool>("use_footprint", use_footprint_);
  getInput<bool>("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  costmap_ = costmap_ros_->getCostmap();
}

BT::NodeStatus IsPoseOccupiedCondition::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
  geometry_msgs::msg::PoseStamped pose;
  getInput("pose", pose);

  unsigned int cost = nav2_costmap_2d::FREE_SPACE;

  unsigned int mx = 0;
  unsigned int my = 0;

  if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    cost = costmap_->getCost(mx, my);
  } 
  else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Pose (%.2f, %.2f) is out of costmap bounds",
      pose.pose.position.x, pose.pose.position.y);
    return BT::NodeStatus::FAILURE;
  }

  if ((cost == 255 && !consider_unknown_as_obstacle_) || cost < cost_threshold_) {
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPoseOccupiedCondition>("IsPoseOccupied");
}
