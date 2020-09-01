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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "nav2_msgs/msg/context_info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_msgs/msg/context_info.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

#define NARROW_THRLD 20

class ContextDetector : public rclcpp::Node
{
public:
  ContextDetector(const std::string & name)
  : Node(name)
  {
    marker_counter_ = 0;
  }

  void init()
  {
    map_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(1).transient_local().reliable().keep_all(),
      std::bind(&ContextDetector::map_cb, this, _1));

    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      rclcpp::QoS(1).transient_local().reliable().keep_all(),
      std::bind(&ContextDetector::global_costmap_cb, this, _1));

    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap",
      rclcpp::QoS(1).transient_local().reliable().keep_all(),
      std::bind(&ContextDetector::local_costmap_cb, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&ContextDetector::scan_cb, this, _1));

    diff_costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/context/diff_costmap",
      rclcpp::QoS(1).transient_local().reliable().keep_all());
    
    context_pub_ = create_publisher<nav2_msgs::msg::ContextInfo>(
      "context_update",
      rclcpp::SystemDefaultsQoS());
    
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "context_marker",
      100);
    
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan",
      rclcpp::QoS(5).durability_volatile().reliable().keep_last(5),
      std::bind(&ContextDetector::plan_cb, this, _1));
    
    timer_ = create_wall_timer(
      500ms, std::bind(&ContextDetector::timer_callback, this));  
      
    tfBuffer_ = std::make_shared<tf2::BufferCore>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, shared_from_this(), false);
  }

private:

  unsigned char interpretValue(unsigned char value)
  {
    // check if the static value is above the unknown or lethal thresholds
    if (value == 255) {
      return nav2_costmap_2d::NO_INFORMATION;
    } else if (value >= 100) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    } else {
      return nav2_costmap_2d::FREE_SPACE;
    }

    double scale = static_cast<double>(value) / 100.0;
    return scale * nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2D> create_costmap(const nav_msgs::msg::OccupancyGrid & msg) const
  {
    unsigned int size_x = msg.info.width;
    unsigned int size_y = msg.info.height;
    
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
      size_x, size_y,
      msg.info.resolution,
      msg.info.origin.position.x,  msg.info.origin.position.y);

    unsigned int index = 0;

    // initialize the costmap with static data
    for (unsigned int i = 0; i < size_y; ++i) {
      for (unsigned int j = 0; j < size_x; ++j) {
        unsigned char value = msg.data[index];
        costmap->setCost(i, j, value);
        ++index;
      }
    }

    return costmap;
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_msg_ = *msg;
  }

  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = *msg;

    map_costmap_ = create_costmap(*msg);
  }

  void global_costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    global_msg_ = *msg;

    if (global_costmap_ != nullptr && last_global_costmap_ == nullptr) {
      last_global_costmap_ = global_costmap_;
    }

    global_costmap_ = create_costmap(*msg);
  }

  void local_costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    local_msg_ = *msg;

    local_costmap_ = create_costmap(*msg);
  }

  void plan_cb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::cerr << "=================" << std::endl;
    path_ = *msg;
  }

  void publish_marker_context(const std::string & context)
  {    
    if (!tfBuffer_->canTransform("base_link", "map", tf2::TimePointZero)) {
      std::cerr << "No transform" << std::endl;
      return;
    }

    auto l2g_msg = tfBuffer_->lookupTransform("map", "base_link", tf2::TimePointZero);


    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.id = marker_counter_++;
    marker.lifetime = rclcpp::Duration(0);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = l2g_msg.transform.translation.x;
    marker.pose.position.y = l2g_msg.transform.translation.y;
    marker.pose.position.z = l2g_msg.transform.translation.z;
    marker.pose.orientation = l2g_msg.transform.rotation;
    marker.scale.x = 0.2;
    marker.scale.y = 0.02;
    marker.scale.z = 0.01;

    if (context == "narrow") {
      marker.color.r = 1.0;
    } else if (context == "dynamic") {
      marker.color.b = 1.0;
    } else {
      marker.color.g = 1.0;
    }
    marker.color.a = 1.0;

    marker_pub_->publish(marker);
  }

  void timer_callback()
  {
    nav2_msgs::msg::ContextInfo context_msg;
    context_msg.context_state = "normal";
    if (global_costmap_ != nullptr) {
      if ((now() - rclcpp::Time(path_.header.stamp)).seconds() <= 1.5) {
        if (is_narrow_space(global_costmap_, path_, 1.0)) {
          std::cerr << "Narrow space" << std::endl;
          context_msg.context_state = "narrow";
        } else {
          std::cerr << "Open space" << std::endl;
        }
      } else {
        std::cerr << "No path " << (now() - rclcpp::Time(path_.header.stamp)).seconds() << std::endl;
      }
    } else {
      std::cerr << "No global costmap" << std::endl;
    }

    if (local_costmap_ != nullptr) {
      if (is_dynamic_space()) {
        std::cerr << "Dynamic space" << std::endl;
        context_msg.context_state = "dynamic";
      } else {
        std::cerr << "Static space" << std::endl;
      }
    } else {
      std::cerr << "No local costmap" << std::endl;
    }

    publish_marker_context(context_msg.context_state);

    context_pub_->publish(context_msg);
  }

  // op1 diff op2
  nav2_costmap_2d::Costmap2D diff_costmap(const nav_msgs::msg::OccupancyGrid & op1, const nav_msgs::msg::OccupancyGrid & op2)
  {
    auto costmap_1 = create_costmap(op1);
    auto costmap_2 = create_costmap(op2);
    nav2_costmap_2d::Costmap2D ret(*costmap_1);

    if (!tfBuffer_->canTransform(op1.header.frame_id, op2.header.frame_id, tf2::TimePointZero)) {
      std::cerr << "No transform" << std::endl;
      return ret;
    }

    auto l2g_msg = tfBuffer_->lookupTransform(op1.header.frame_id, op2.header.frame_id, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> l2g;
    tf2::fromMsg(l2g_msg, l2g);

    for (int i = 0; i < costmap_1->getSizeInCellsX(); i++) {
      for (int j = 0; j < costmap_1->getSizeInCellsY(); j++) {
        auto op1_cost = costmap_1->getCost(j ,i);

        double xl, yl;
        costmap_1->mapToWorld(i, j, xl, yl);
        tf2::Vector3 point_costmap1(xl, yl, 0.0);
        tf2::Vector3 point_costmap2 = l2g * point_costmap1;

        unsigned int mx, my;
        costmap_2->worldToMap(point_costmap2.x(), point_costmap2.y(), mx, my);
        auto op2_cost = costmap_2->getCost(my, mx);

        if (op2_cost )
        ret.setCost(j, i, abs(op1_cost - op2_cost));
      }
    }
    return ret;
  }

  // op1 - op2
  nav2_costmap_2d::Costmap2D substract_costmap(const nav_msgs::msg::OccupancyGrid & op1, const nav_msgs::msg::OccupancyGrid & op2)
  {
    auto costmap_1 = create_costmap(op1);
    auto costmap_2 = create_costmap(op2);
    nav2_costmap_2d::Costmap2D ret(*costmap_1);

    if (!tfBuffer_->canTransform(op1.header.frame_id, op2.header.frame_id, tf2::TimePointZero)) {
      std::cerr << "No transform" << std::endl;
      return ret;
    }

    auto l2g_msg = tfBuffer_->lookupTransform(op1.header.frame_id, op2.header.frame_id, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> l2g;
    tf2::fromMsg(l2g_msg, l2g);

    for (int i = 0; i < costmap_1->getSizeInCellsX(); i++) {
      for (int j = 0; j < costmap_1->getSizeInCellsY(); j++) {
        auto op1_cost = costmap_1->getCost(j ,i);

        double xl, yl;
        costmap_1->mapToWorld(i, j, xl, yl);
        tf2::Vector3 point_costmap1(xl, yl, 0.0);
        tf2::Vector3 point_costmap2 = l2g * point_costmap1;

        unsigned int mx, my;
        costmap_2->worldToMap(point_costmap2.x(), point_costmap2.y(), mx, my);
        auto op2_cost = costmap_2->getCost(my, mx);

        ret.setCost(j, i, std::max(0, op1_cost - op2_cost));
      }
    }
    return ret;
  }

  nav2_costmap_2d::Costmap2D inflate_costmap(nav2_costmap_2d::Costmap2D & input, int inflate)
  {
    nav2_costmap_2d::Costmap2D ret(input);

    for (int i = 0; i < ret.getSizeInCellsX(); i++) {
      for (int j = 0; j < ret.getSizeInCellsY(); j++) {
        
        if (input.getCost(i, j) > 0) {
          for (int ii = -inflate; ii <= inflate; ii++) {
            for (int ij = -inflate; ij <= inflate; ij++) {
              if( (i + ii) >= 0 && (i + ii) < ret.getSizeInCellsX() && (j + ij) >= 0 && (j + ij) < ret.getSizeInCellsY())
                ret.setCost(i + ii, j + ij, 254u);
            }
          }
        }
      }
    }


    return ret;
  }

  nav2_costmap_2d::Costmap2D get_costmap_from_scan(const nav_msgs::msg::OccupancyGrid & layout)
  {
    nav2_costmap_2d::Costmap2D ret(*create_costmap(layout));
    for (int i = 0; i < ret.getSizeInCellsX(); i++) {
      for (int j = 0; j < ret.getSizeInCellsY(); j++) {
        ret.setCost(j ,i, 0);
      }
    }

    if (!tfBuffer_->canTransform(layout.header.frame_id, scan_msg_.header.frame_id, tf2::TimePointZero)) {
      std::cerr << "No transform" << std::endl;
      return ret;
    }

    auto scan2costmap_msg = tfBuffer_->lookupTransform(layout.header.frame_id, scan_msg_.header.frame_id, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> scan2costmap;
    tf2::fromMsg(scan2costmap_msg, scan2costmap);

    double angle =  scan_msg_.angle_min;
    int counter = 0;
    while (angle < scan_msg_.angle_max && counter < scan_msg_.ranges.size()) {
      tf2::Vector3 p_scan(
        scan_msg_.ranges[counter] * cos(angle),
        scan_msg_.ranges[counter] * sin(angle),
        0.0);
      tf2::Vector3 p_costmap = scan2costmap * p_scan;

      unsigned int mx, my;
      if (ret.worldToMap(p_costmap.x(), p_costmap.y(), mx, my)) {
        // const int inflate = 2;
        // for (int i = -inflate; i <= inflate; i++) {
        //   for (int j = -inflate; j <= inflate; j++) {
        //     if( (my + i) >= 0 && (my + i) < ret.getSizeInCellsX() && (mx + j) >= 0 && (mx + j) < ret.getSizeInCellsY())
        //       ret.setCost(my + i, mx + j, 255);
        //   }
        // }
        ret.setCost(my, mx, 255u);
      }

      angle += scan_msg_.angle_increment;
      counter++;
    }

    return ret;

  }

  nav2_costmap_2d::Costmap2D get_cropped_map(
    const nav_msgs::msg::OccupancyGrid & input,
    const nav_msgs::msg::OccupancyGrid & layout)
  {
    nav2_costmap_2d::Costmap2D ret(*create_costmap(layout));

    if (!tfBuffer_->canTransform(layout.header.frame_id, input.header.frame_id, tf2::TimePointZero)) {
      std::cerr << "No transform" << std::endl;
      return ret;
    }

    auto layout2input_msg = tfBuffer_->lookupTransform(input.header.frame_id, layout.header.frame_id, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> layout2input;
    tf2::fromMsg(layout2input_msg, layout2input);

    for (int i = 0; i < ret.getSizeInCellsX(); i++) {
      for (int j = 0; j < ret.getSizeInCellsY(); j++) {
        double xl, yl;
        ret.mapToWorld(i, j, xl, yl);

        tf2::Vector3 point_layout(xl, yl, 0.0);
        tf2::Vector3 point_input = layout2input * point_layout;

        unsigned int mx, my;
        map_costmap_->worldToMap(point_input.x(), point_input.y(), mx, my);
        auto map_cost = map_costmap_->getCost(my, mx);

        ret.setCost(j, i, map_cost);
      }
    }

    return ret;
  }

  bool is_dynamic_space()
  {
    if (last_global_costmap_ == nullptr) {
      std::cerr << "No last costmap" << std::endl;
      return false;
    }
  
    if (!tfBuffer_->canTransform("base_link", "map", tf2::TimePointZero)) {
      std::cerr << "No transform" << std::endl;
      return false;
    }

    auto l2g_msg = tfBuffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

    const double dist_scan = 2.0;

    unsigned int mx_min, my_min;
    unsigned int mx_max, my_max;

    global_costmap_->worldToMap(
      l2g_msg.transform.translation.x - dist_scan, 
      l2g_msg.transform.translation.y - dist_scan,
      mx_min, my_min);
 
    mx_min = std::max(mx_min, 0u);
    my_min = std::max(my_min, 0u);

    global_costmap_->worldToMap(
      l2g_msg.transform.translation.x + dist_scan, 
      l2g_msg.transform.translation.y + dist_scan,
      mx_max, my_max);
 
    mx_max = std::min(mx_max, global_costmap_->getSizeInCellsX());
    my_max = std::min(my_max, global_costmap_->getSizeInCellsY());

    nav2_costmap_2d::Costmap2D dynamic_costmap(*global_costmap_);
    // dynamic_costmap.resetMap(0u, 0u, dynamic_costmap.getSizeInCellsX(), dynamic_costmap.getSizeInCellsY());
    for (unsigned int i = 0; i < dynamic_costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < dynamic_costmap.getSizeInCellsY(); ++j) {
        dynamic_costmap.setCost(i, j, 0);
      }
    }

   //  std::cerr << "[" << mx_min << " - " << mx_max << ", " << my_min << " - " << my_max << "]" << std::endl; 

    int counter = 0;
    for (unsigned int x = mx_min; x < mx_max; x++) {
      for (unsigned int y = my_min; y < my_max; y++) {
        auto last_value = last_global_costmap_->getCost(x, y);
        auto current_value = global_costmap_->getCost(x, y);

        if ((last_value < 10u) && current_value > 20u) {
          counter++;
          dynamic_costmap.setCost(x, y, current_value);
        }
      }
    }

    diff_costmap_pub_->publish(get_occupancy_from_costmap(dynamic_costmap, map_msg_));

    return counter > 20;
    /*
    nav2_costmap_2d::Costmap2D laser_costmap = get_costmap_from_scan(local_msg_);
    nav2_costmap_2d::Costmap2D local_map_costmap = get_cropped_map(map_msg_, local_msg_);
    
    nav2_costmap_2d::Costmap2D inflate_map = inflate_costmap(local_map_costmap, 4);

     nav2_costmap_2d::Costmap2D subtraction_costmap = substract_costmap(
      get_occupancy_from_costmap(laser_costmap, local_msg_), 
      get_occupancy_from_costmap(inflate_map, local_msg_));    

    int dynamic_counter = 0;
    for (unsigned int i = 0; i < subtraction_costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < subtraction_costmap.getSizeInCellsY(); ++j) {
        if(subtraction_costmap.getCost(i, j) > 10) {
          std::cerr << "  v = " << static_cast<int>(subtraction_costmap.getCost(i, j)) << std::endl;
          dynamic_counter++;
        }
      }
    }

    std::cerr << "Counter = " << dynamic_counter << std::endl;

    diff_costmap_pub_->publish(get_occupancy_from_costmap(subtraction_costmap, local_msg_));

    return dynamic_counter > 5;*/



  }

  nav_msgs::msg::OccupancyGrid get_occupancy_from_costmap(const nav2_costmap_2d::Costmap2D & costmap, const nav_msgs::msg::OccupancyGrid & layout) 
  {
    nav_msgs::msg::OccupancyGrid ret = layout;
    
    if (ret.data.size() != costmap.getSizeInCellsX() * costmap.getSizeInCellsY()) {
      RCLCPP_ERROR(get_logger(), "get_occupancy_from_costmap: Size mismatch %zu != %zu", 
        ret.data.size(), costmap.getSizeInCellsX() * costmap.getSizeInCellsY());
      return ret;
    }

    unsigned int index = 0;
    // initialize the costmap with static data
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i) {
      for (unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j) {
        ret.data[index] = costmap.getCost(i, j);
        ++index;
      }
    }

    return ret;
  }

  bool is_narrow_space(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap, nav_msgs::msg::Path & path_, double dist = 1.0) 
  {
    bool is_narrow = false;
    double dist_acum = 0.0;

    auto current_pose_it = path_.poses.begin();
    while (!is_narrow && dist_acum < dist && current_pose_it != path_.poses.end()) {
      auto pose = *current_pose_it;

      unsigned int mx, my;
      costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
      
      if (costmap->getCost(mx, my) > NARROW_THRLD) {
        is_narrow = true;
      }

      current_pose_it++;
      auto next_pose = *current_pose_it;

      double dist_x = fabs(pose.pose.position.x - next_pose.pose.position.x);
      double dist_y = fabs(pose.pose.position.y - next_pose.pose.position.y);

      dist_acum += sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    return is_narrow;
  }


private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr diff_costmap_pub_;
  rclcpp::Publisher<nav2_msgs::msg::ContextInfo>::SharedPtr context_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  nav_msgs::msg::OccupancyGrid map_msg_, local_msg_, global_msg_;
  sensor_msgs::msg::LaserScan scan_msg_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> last_global_costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> global_costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> local_costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> map_costmap_;
  nav_msgs::msg::Path path_;

  std::shared_ptr<tf2::BufferCore> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int marker_counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto context_detector_node = std::make_shared<ContextDetector>("clicked_point_to_pose");
  context_detector_node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context_detector_node);
 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}