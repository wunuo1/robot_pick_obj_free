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

  target_subscriber_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "racing_obstacle_detection",
      10,
      std::bind(&PickObjFreeNode::subscription_callback_target,
      this,
      std::placeholders::_1));
    order_interpreter_ = std::make_shared<OrderInterpreter>();
    arm_pose_solver_ = std::make_shared<ArmPoseSolver3Dof>(28, 60, 85);
}

PickObjFreeNode::~PickObjFreeNode(){
    
}
void PickObjFreeNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
    static bool successful = false;
    int center_x = 0;
    int center_y = 0;
    if(successful == false){
        for(const auto &target : targets_msg->targets){
            if(target.type == "red_ball"){
                if(target.rois[0].confidence > 0.5){
                    center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
                    center_y = target.rois[0].rect.y_offset + target.rois[0].rect.height / 2;
                }
            }
        }
        if((center_x != 0) || (center_y != 0)){
            CenterCoordinate center_coordinate = {center_x, center_y};
            centers_queue.push_back(center_coordinate);

            if(centers_queue.size() == 5){
                if(is_stable() == true){
                    JointAngles result = arm_pose_solver_->pose_calculation(center_x, center_y);
                    if(!std::isnan(result.theta3) && !std::isnan(result.theta2) && !std::isnan(result.theta1)){
                        int joint_6_weight = 500 - ((result.theta3 * 180 / 3.1459) * 1000 / 240);
                        int joint_7_weight = 500 - ((result.theta2  * 180 / 3.1459  - 90) *  1000 / 240);
                        int joint_8_weight = 350 + ((result.theta1 * 180 / 3.1459) *  1000 / 240 );
                        order_interpreter_->control_serial_servo(6, joint_6_weight, 2000);
                        order_interpreter_->control_serial_servo(7, joint_7_weight, 2000);
                        order_interpreter_->control_serial_servo(8, joint_8_weight, 2000);
                        successful = true;
                    } else {
                        std::cout<<"Having nan in numbers"<<std::endl;
                    }
                } else {
                    std::cout<<"Unstable detection"<<std::endl;
                }
                centers_queue.pop_front();
            }
        }
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

  rclcpp::spin(std::make_shared<PickObjFreeNode>("PickObjFreeNode"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("PickObjFreeNode"), "Pkg exit.");
  return 0;
}