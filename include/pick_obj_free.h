// Copyright (c) 2022，Horizon Robotics.
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

#ifndef PICK_OBJ_FREE_H_
#define PICK_OBJ_FREE_H_

#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "order_interpreter.hpp"
#include "arm_pose_solver.hpp"
#include "ai_msgs/msg/perception_targets.hpp"

struct CenterCoordinate {
    int center_x, center_y;
};


class PickObjFreeNode : public rclcpp::Node{
public:
  PickObjFreeNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PickObjFreeNode() override;
private:

    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;

    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    bool is_stable();
    std::deque<CenterCoordinate> centers_queue;
    std::shared_ptr<ArmPoseSolver3Dof> arm_pose_solver_;
    std::shared_ptr<OrderInterpreter> order_interpreter_;
};


#endif  // PICK_OBJ_FREE_H_