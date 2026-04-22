#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), global_map_(width_ * height_, -1) {}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid& msg) {
  latest_costmap_ = msg;
  has_costmap_ = true;
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry& msg) {
  robot_x_ = msg.pose.pose.position.x;
  robot_y_ = msg.pose.pose.position.y;

  // extract yaw from quarternion
  const auto& q = msg.pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool MapMemoryCore::tryMerge() {
  if (!has_costmap_) return false;

  if (has_last_) {
    double dx = robot_x_ - last_x_;
    double dy = robot_y_ - last_y_;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < distance_threshold_) return false;
  }

  mergeLatestCostmap();

  last_x_ = robot_x_;
  last_y_ = robot_y_;
  has_last_ = true;
  return true;
}

void MapMemoryCore::mergeLatestCostmap() {
  const auto& cm = latest_costmap_;
  const double cm_res = cm.info.resolution;
  const int cm_w = cm.info.width;
  const int cm_h = cm.info.height;
  const double cm_ox = cm.info.origin.position.x;
  const double cm_oy = cm.info.origin.position.y;

  // caching these outside of loop -> 2 calls instead of 80000 per merge
  const double cos_y = std::cos(robot_yaw_);
  const double sin_y = std::sin(robot_yaw_);

  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      int8_t value = cm.data[cy * cm_w + cx];
      if (value < 0) continue; 

      // step 1: cell -> local metres (centre of cell)
      double local_x = cm_ox + (cx + 0.5) * cm_res;
      double local_y = cm_oy + (cy + 0.5) * cm_res;

      // step 2: local -> world (rotate by yaw, translate by robot pose)
      double world_x = robot_x_ + local_x * cos_y - local_y * sin_y;
      double world_y = robot_y_ + local_x * sin_y + local_y * cos_y;

      // step 3: world -> global grid
      int g_x = static_cast<int>((world_x - origin_x_) / resolution_);
      int g_y = static_cast<int>((world_y - origin_y_) / resolution_);

      // step 4: bounds check
      if (g_x < 0 || g_x >= width_ || g_y < 0 || g_y >= height_) continue;

      // merge: keep max ie obstacles win over free space
      if (value > global_map_[g_y * width_ + g_x]) {
        global_map_[g_y * width_ + g_x] = value;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(global_map_.begin(), global_map_.end());
  return msg;
}

} 
