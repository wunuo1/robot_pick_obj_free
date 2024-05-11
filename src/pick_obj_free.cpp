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

#include "pick_obj_free.h"
#include "order_interpreter.hpp"

#include <cmath>
#include <numeric>


PickObjFreeNode::PickObjFreeNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) {

    this->declare_parameter<bool>("task_input", task_input_);
    this->get_parameter<bool>("task_input", task_input_);

    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber_;

    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if(task_input_ == true) {
        task_server_ = this->create_service<robot_pick_obj_msg::srv::TaskExecution>(
            "/task",
            std::bind(&PickObjFreeNode::service_callback_task, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_default, callback_group_service_);

        detecting_ = false;
        std::cout << "Waiting for request" << std::endl;
    }


    target_subscriber_ =
        this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "racing_obstacle_detection",
        1,
        std::bind(&PickObjFreeNode::subscription_callback_target,
        this,
        std::placeholders::_1), sub_opt);

    mutex_ = std::make_shared<std::mutex>();
    condition_variable_ = std::make_shared<std::condition_variable>();
    order_interpreter_ = std::make_shared<OrderInterpreter>();
    arm_pose_solver_ = std::make_shared<ArmPoseSolver3Dof>(28, 60, 85, 137 + 8);
    order_interpreter_->control_serial_servo(7, 500, 1000);
    order_interpreter_->control_serial_servo(6, 500, 1000);
    order_interpreter_->control_serial_servo(8, 350, 1000);
    // order_interpreter_->control_PWM_servo(1, 1100, 200);
}

PickObjFreeNode::~PickObjFreeNode(){
    
}
void PickObjFreeNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){

    if(task_input_ == true){
        std::unique_lock<std::mutex> lock(*mutex_);
        if(detecting_ == true){
            target_process(targets_msg);
        }
    } else {
        if(detecting_ == true){
            target_process(targets_msg);
        }
    }
}

void PickObjFreeNode::service_callback_task(const robot_pick_obj_msg::srv::TaskExecution::Request::SharedPtr task_request,
                            const robot_pick_obj_msg::srv::TaskExecution::Response::SharedPtr task_response){
    if (target_values_.find(task_request->target_type) == target_values_.end()) {
        std::cout << "target type is invalid: " << task_request->target_type << std::endl;
        task_response->successful = false;
    }
    if (task_values_.find(task_request->task_type) == task_values_.end()) {
        std::cout << "task type is invalid:  " << task_request->task_type << std::endl;
        task_response->successful = false;
    }
    target_type_ = task_request->target_type;
    task_type_ = task_request->task_type;
    {
        std::unique_lock<std::mutex> lock(*mutex_);
        detecting_ = true;
        condition_variable_->wait(lock);
        task_response->successful = true;
    }
    task_response->successful = true;
}


void PickObjFreeNode::target_process(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
    int center_x = 0;
    int center_y = 0;
    for(const auto &target : targets_msg->targets){
        if(target.type == target_type_){
            if(target.rois[0].confidence > 0.5){
                center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
                center_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
            }
        }
    }
    std::cout<<"center_y______:"<<center_y<<std::endl;
    if((center_x != 0) || (center_y != 0)){
        if(center_x > 300) {
            order_interpreter_->control_serial_servo("right_move_hand_free");
            return;
        }
        if(center_x < 50) {
            order_interpreter_->control_serial_servo("left_move_hand_free");
            return;
        }
        if(center_y < 320) {
            order_interpreter_->control_serial_servo("go_forward_hand_free");
            return;
        }
        if(center_y > 420) {
            order_interpreter_->control_serial_servo("back_small_step_hand_free");
            return;
        }
        CenterCoordinate center_coordinate = {center_x, center_y};
        centers_queue.push_back(center_coordinate);
        if(centers_queue.size() == 5){
            if(is_stable() == true){
                
                if(task_type_ == "catch") {
                    order_interpreter_->control_serial_servo(18, 800, 2000);
                }
                std::cout<<"center_x: "<<center_x<<std::endl;
                std::cout<<"center_y: "<<center_y<<std::endl;
                if(move_hand_to_target(center_x, center_y) == true){
                    if(task_type_ == "catch") {
                        order_interpreter_->control_serial_servo(18, 650, 2000);
                    } else {
                        order_interpreter_->control_serial_servo(18, 800, 1000);
                    }
                    order_interpreter_->control_serial_servo(8, 350, 1000);
                    order_interpreter_->control_serial_servo(6, 500, 1000);
                    order_interpreter_->control_serial_servo(7, 500, 1000);
                    detecting_ = false;
                    if(task_input_ == true) condition_variable_->notify_one();
                }
            } else {
                std::cout<<"Unstable detection"<<std::endl;
            }
            centers_queue.pop_front();
        }
    } else {
        std::cout<<"No target"<<std::endl;
    }
}


bool PickObjFreeNode::move_hand_to_target(const int& center_x, const int& center_y){
    int height = 20;
    int increased_height = 5;
    if(target_type_ == "base"){
        increased_height = 50;
        height = 20;
    }
    JointAngles result = arm_pose_solver_->pose_calculation(center_x, center_y, height, increased_height);
    if(!std::isnan(result.theta3) && !std::isnan(result.theta2) && !std::isnan(result.theta1)){
        int joint_6_weight = 500 - ((result.theta3 * 180 / 3.1459) * 1000 / 240);
        int joint_7_weight = 500 - ((result.theta2  * 180 / 3.1459  - 90) *  1000 / 240);
        int joint_8_weight = 350 + ((result.theta1 * 180 / 3.1459) *  1000 / 240 );
        order_interpreter_->control_serial_servo(6, joint_6_weight, 1000);
        order_interpreter_->control_serial_servo(7, joint_7_weight, 1000);
        order_interpreter_->control_serial_servo(8, joint_8_weight, 2000);
        return true;
    } else {
        std::cout<<"Having nan in numbers"<<std::endl;
        return false;
    }
}

bool PickObjFreeNode::is_stable(){
    int x_sum = 0;
    int y_sum = 0;
    double x_mean = 0.0;
    double y_mean = 0.0;
    for(int i = 0; i < 5; i++){
        x_sum = x_sum + centers_queue[i].center_x;
        y_sum = y_sum + centers_queue[i].center_y;
    }
    x_mean = x_sum / 5;
    y_mean = y_sum / 5;
    int x_squares_sum = 0;
    int y_squares_sum = 0;
    for(int i = 0; i < 5; i++){
        x_squares_sum = x_squares_sum + (centers_queue[i].center_x - x_mean) * (centers_queue[i].center_x - x_mean);
        y_squares_sum = y_squares_sum + (centers_queue[i].center_y - y_mean) * (centers_queue[i].center_y - y_mean);
    }
    double x_std_dev = sqrt(x_squares_sum / 5);
    double y_std_dev = sqrt(y_squares_sum / 5);

    if((x_std_dev <= 2) && (y_std_dev <=2)){
        return true;
    } else{
        return false;
    }
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickObjFreeNode>("PickObjFreeNode");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("PickObjFreeNode"), "Pkg exit.");
  return 0;
}