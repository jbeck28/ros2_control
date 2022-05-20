// Copyright 2020 ROS2-Control Development Team
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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";


  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // When the MutliThreadedExecutor issues are fixed (ros2/rclcpp#1168), this loop should be
  // converted back to a timer.
  std::thread cm_thread([cm]() {
    RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr calc_time_pub_;
    publisher_ = cm->create_publisher<std_msgs::msg::Float64>("loop_period", 10);
    calc_time_pub_ = cm->create_publisher<std_msgs::msg::Float64>("update_time", 10);
    realtime_tools::RealtimePublisher<std_msgs::msg::Float64> rt_pub(publisher_);
    realtime_tools::RealtimePublisher<std_msgs::msg::Float64> rt_calc_time_pub(calc_time_pub_);

    rclcpp::Time current_time = cm->now();
    rclcpp::Time previous_time = current_time;
    rclcpp::Time end_period = current_time;
    rclcpp::Time calc_time_start;
    rclcpp::Time calc_time_end;
    // Use nanoseconds to avoid chrono's rounding
    rclcpp::Duration period(std::chrono::nanoseconds(1000000000 / cm->get_update_rate()));

    while (rclcpp::ok())
    {
      // wait until we hit the end of the period
      end_period += period;
      std::this_thread::sleep_for(std::chrono::nanoseconds((end_period - cm->now()).nanoseconds()));

      // execute update loop
      auto period = current_time - previous_time;
      cm->read(current_time, period);
      current_time = cm->now();


      cm->update(current_time, period);


      calc_time_start = cm->now();
      
      calc_time_end = cm->now();
      if (rt_pub.trylock()){
        rt_pub.msg_.data = rclcpp::Duration(current_time-previous_time).seconds();
        rt_pub.unlockAndPublish();
      }
      if(rt_calc_time_pub.trylock()){
        rt_calc_time_pub.msg_.data = rclcpp::Duration(calc_time_start - calc_time_end).seconds();
        rt_calc_time_pub.unlockAndPublish();
      }
      
      previous_time = current_time;
      cm->write(current_time, period);
    }
  });
   pthread_t this_thread = cm_thread.native_handle();
    int ret;
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    std::cout <<"Attemping to set thread realtime prio = " << params.sched_priority <<std::endl;

    // Attempt to set thread real-time priority to the SCHED_FIFO policy
    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
    if (ret != 0) {
        // Print the error
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        //return;     
    }else{
      std::cout <<"Successfully set thread priority" << std::endl;
    }
  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
