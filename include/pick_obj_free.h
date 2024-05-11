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
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "order_interpreter.hpp"
#include "arm_pose_solver.hpp"

#include "ai_msgs/msg/perception_targets.hpp"
#include "robot_pick_obj_msg/srv/task_execution.hpp"

struct CenterCoordinate {
    int center_x, center_y;
};


class PickObjFreeNode : public rclcpp::Node{
public:
  PickObjFreeNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PickObjFreeNode() override;
private:

    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_ = nullptr;
    rclcpp::Service<robot_pick_obj_msg::srv::TaskExecution>::SharedPtr task_server_ = nullptr;

    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void service_callback_task(const robot_pick_obj_msg::srv::TaskExecution::Request::SharedPtr task_request,
                                const robot_pick_obj_msg::srv::TaskExecution::Response::SharedPtr task_response);

    void target_process(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    bool move_hand_to_target(const int& center_x, const int& center_y);
    bool is_stable();
    std::deque<CenterCoordinate> centers_queue;
    std::shared_ptr<ArmPoseSolver3Dof> arm_pose_solver_;
    std::shared_ptr<OrderInterpreter> order_interpreter_;

    bool task_input_ = false;
    bool detecting_ = true;

    std::string target_type_ = "red_ball";
    std::string task_type_ = "catch";

    std::shared_ptr<std::condition_variable> condition_variable_;
    std::shared_ptr<std::mutex> mutex_;

    std::unordered_set<std::string> target_values_ = {"red_ball", "green_ball", "blue_ball", "base"};
    std::unordered_set<std::string> task_values_ = {"catch", "put"};

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_ = nullptr;
    rclcpp::CallbackGroup::SharedPtr callback_group_service_ = nullptr;


};


#endif  // PICK_OBJ_FREE_H_