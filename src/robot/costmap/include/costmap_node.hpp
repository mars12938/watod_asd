#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"

using namespace std;
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishMessage();
    vector<vector<int>> grid{200, vector<int>{200, -1}};
    void publishMap(const sensor_msgs::msg::LaserScan::SharedPtr scan);
 
  private:

    robot::CostmapCore costmap_;
    // Place these constructs here
    //
    //
    // subscription ptr
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 
