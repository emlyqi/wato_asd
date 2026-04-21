#include "costmap_core.hpp"

#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger), grid_(width_ * height_, 0) {}


void CostmapCore::initializeCostmap() {
  grid_.assign(width_ * height_, 0);
}

bool CostmapCore::convertToGrid(double range, double angle, int& x_cell, int& y_cell) const {
  double x_world = range * std::cos(angle);
  double y_world = range * std::sin(angle);

  x_cell = static_cast<int>((x_world - origin_x_) / resolution_);
  y_cell = static_cast<int>((y_world - origin_y_) / resolution_);

  return (x_cell >= 0 && x_cell < width_ && y_cell >= 0 && y_cell < height_);
}

void CostmapCore::markObstacle(int x_cell, int y_cell) {
  grid_[y_cell * width_ + x_cell] = max_cost_;
}

void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan& scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    double angle = scan.angle_min + i * scan.angle_increment;

    if (range < scan.range_min || range > scan.range_max || std::isnan(range)) {
      continue;
    }

    int x_cell, y_cell;
    if (convertToGrid(range, angle, x_cell, y_cell)) {
      markObstacle(x_cell, y_cell);
    }
  }

  inflateObstacles();
}

void CostmapCore::inflateObstacles() {
  // snapshot obstacle cells before inflating so we don't cascade
  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y * width_ + x] == max_cost_) {
        obstacles.emplace_back(x, y);
      }
    }
  }

  // radius in cells (e.g. 1.0m / 0.1m = 10 cells)
  int radius_cells = static_cast<int>(inflation_radius_ / resolution_);

  // for each obstacle, paint halo
  for (const auto& [ox, oy] : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int nx = ox + dx;
        int ny = oy + dy;

        // skip out-of-bounds
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        double dist_cells = std::sqrt(dx * dx + dy * dy);
        double dist_meters = dist_cells * resolution_;
        if (dist_meters > inflation_radius_) continue;

        int new_cost = static_cast<int>(max_cost_ * (1.0 - dist_meters / inflation_radius_));

        // only write if new cost beats existing cost
        int idx = ny * width_ + nx;
        if (new_cost > grid_[idx]) {
          grid_[idx] = new_cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0; // identity rotation (no rotation)
  msg.data.assign(grid_.begin(), grid_.end()); // copy grid into msg
  return msg;
}
}