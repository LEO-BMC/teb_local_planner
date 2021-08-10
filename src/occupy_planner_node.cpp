#include <memory>
#include <ros/ros.h>
#include "teb_local_planner/occupy_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "occupy_planner_exe");
  auto node_handle_ptr = std::make_shared<ros::NodeHandle>("~");
  occupy_planner::OccupyPlanner occupy_planner_object(node_handle_ptr);
  ros::spin();
  std::cout << "occupy_planner_exe has ended." << std::endl;
  return 0;
}
