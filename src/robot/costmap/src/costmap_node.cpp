#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

double Euclidean (int x1, int y1, int x2, int y2) {
    return std::sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));

}

void CostmapNode::publishMap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 3: Inflate obstacles
    //inflateObstacles();
    auto grid_message = nav_msgs::msg::OccupancyGrid();

    // flatten
    std::vector<int8_t> flatten;
    for (int n=0; n<grid.size(); n++){
        for (int m=0; m<grid.size(); m++)
        {
            if(grid[n][m] > 100){
                flatten.push_back(0);
            } 
            if(grid[n][m] < 0)
            {
                flatten.push_back(0);
            } else {

                flatten.push_back(grid[n][m]);
            }

        }
    }
    grid_message.data = flatten;
    grid_message.header = scan->header;
    grid_message.header.stamp = rclcpp::Clock().now();
    grid_message.info.resolution = 0.1;
    grid_message.info.origin.position.x = 0.0;
    grid_message.info.origin.position.y = 0.0;

    grid_message.info.width = grid.size();
    grid_message.info.height = grid.size();
    RCLCPP_INFO(this->get_logger(), "Publishing costmap: size = %zu", flatten.size());

    grid_pub_->publish(grid_message);
    // Step 4: Publish costmap
    //publishCostmap();



}

// I should have more functions for everything (and I typically do) however for timing reasons
// I decided it would be easier to put it all into one thing and not deal with headers
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
// Step 1: Initialize costmap
    //initializeCostmap();
    //
    double resolution = 0.1;
    double origin_x = -10.0;
    double origin_y = -10.0;


    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate polar coordinates
            float x_ish = range * cos(angle);
            float y_ish = range * sin(angle);

            RCLCPP_INFO(this->get_logger(), "x and y: '%f' '%f'", x_ish, y_ish);
            
            // calculate grid coordinates (I know static_cast is bad practice but I'm out of ideas)
            int x_grid = static_cast<int>(std::round((x_ish - origin_x) / resolution));
            int y_grid = static_cast<int>(std::round((y_ish - origin_y) / resolution));
            
            RCLCPP_INFO(this->get_logger(), "x2 and y2: '%d' '%d'", x_grid, y_grid);

            //convertToGrid(range, angle, x_grid, y_grid);
            // check bounderies
            if(x_grid >= 0 && x_grid < grid.size() && y_grid >= 0 && y_grid < grid.size()) {
                grid[x_grid][y_grid] = 100;
                int inflation_radius = 1;
                double distance = 0.0;
                int cost = -1;
                double raw_cost = 0.0;
                // check within inflation radius of coordinates
                for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                        // make sure central coordinates isn't being hit
                        if(dx != 0 && dy != 0) {
                            int new_x = x_grid + dx;
                            int new_y = y_grid + dy;
                            RCLCPP_INFO(this->get_logger(), "I'm at: '%d' '%d'", new_x, new_y);

                            distance = Euclidean(x_grid, y_grid, new_x, new_y);
                            RCLCPP_INFO(this->get_logger(), "dist: '%f'", distance);
                            // if cell within range, calculate its cost
                            if(distance <= inflation_radius){
                                raw_cost = (1 - (distance / inflation_radius)) * 100;
                                cost = static_cast<int>(std::round(raw_cost));

                                // Clamp the cost to the range [0, 100]
                                cost = std::min(100, std::max(0, cost));
                            }

                            RCLCPP_INFO(this->get_logger(), "COST: '%d'", cost);
                            // again, check within bounds before assigning cost to cell
                            if(new_x >= 0 && new_x < grid.size() && new_y >= 0 && new_y < grid.size()) {
                                if(grid[new_x][new_y] == -1 || cost > grid[new_x][new_y])
                                    grid[new_x][new_y] = cost;
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Grid index out of bounds: (%d, %d)", new_x, new_y);
                            }
                        }
                    }


                }
            }


            //markObstacle(x_grid, y_grid);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Publishing map");

    publishMap(scan);
 
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());

  rclcpp::shutdown();
  return 0;
}
