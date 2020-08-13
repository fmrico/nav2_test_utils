// Copyright 2020 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class ClickedPointToPose : public rclcpp::Node
{
public:
  ClickedPointToPose(const std::string & name)
  : Node(name)
  {
    sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 10, std::bind(&ClickedPointToPose::callback_clicked, this, _1));
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
  }

private:
  void callback_clicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) const
  {
    geometry_msgs::msg::PoseStamped pose;

    pose.header = msg->header;
    pose.pose.position = msg->point;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    pub_->publish(pose);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto clicked_point_to_pose_node = std::make_shared<ClickedPointToPose>("clicked_point_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(clicked_point_to_pose_node);
 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}