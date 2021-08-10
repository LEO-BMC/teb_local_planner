#ifndef SRC_TEB_LOCAL_PLANNER_SRC_OCCUPY_PLANNER_H_
#define SRC_TEB_LOCAL_PLANNER_SRC_OCCUPY_PLANNER_H_

#include <memory>
#include <string>
#include <ros/ros.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

namespace occupy_planner {

class OccupyPlanner {
 public:
  explicit OccupyPlanner(std::shared_ptr<ros::NodeHandle> nh_ptr);

 private:
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  teb_local_planner::PlannerInterfacePtr planner;
  teb_local_planner::RobotFootprintModelPtr robot_model;
  teb_local_planner::TebVisualizationPtr visual;
  boost::shared_ptr<teb_local_planner::PointObstacle> pobst;
  std::vector<teb_local_planner::ObstaclePtr> obst_vector;
  teb_local_planner::ViaPointContainer via_points;
  teb_local_planner::TebConfig config;
  boost::shared_ptr<dynamic_reconfigure::Server<teb_local_planner::TebLocalPlannerReconfigureConfig> > dynamic_recfg;
  ros::Subscriber custom_obst_sub;
  ros::Subscriber via_points_sub;
  ros::Subscriber clicked_points_sub;
  std::vector<ros::Subscriber> obst_vel_subs;
  unsigned int no_fixed_obstacles;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server;
  std::shared_ptr<ros::Timer> cycle_timer;
  std::shared_ptr<ros::Timer> publish_timer;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::PoseStamped::ConstPtr msg_pose_stamped_start_;
  geometry_msgs::PoseStamped::ConstPtr msg_pose_stamped_goal_;
  geometry_msgs::PoseArray::ConstPtr msg_pose_array_hybrid_astar_;

  std::shared_ptr<ros::Subscriber> sub_pose_stamped_start_;
  std::shared_ptr<ros::Subscriber> sub_pose_stamped_goal_;
  std::shared_ptr<ros::Subscriber> sub_ptr_poses_hybrid_astar_;

  void CB_mainCycle(const ros::TimerEvent &e);
  void CB_publishCycle(const ros::TimerEvent &e);
  void CB_reconfigure(teb_local_planner::TebLocalPlannerReconfigureConfig &reconfig, uint32_t level);
  void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);
  void CreateInteractiveMarker(const double &init_x,
                               const double &init_y,
                               unsigned int id,
                               std::string frame,
                               interactive_markers::InteractiveMarkerServer *marker_server,
                               interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
  void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg);
  void CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg);
  void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr &twist_msg, const unsigned int id);

  bool get_affine(Eigen::Affine3f &affine,
                  const std::string &frame_source,
                  const std::string &frame_target,
                  const ros::Time &time);

  void callback_pose_stamped_start(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stampeXd_start);
  void callback_pose_stamped_goal(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal);
  void callback_pose_array_hybrid_astar(const geometry_msgs::PoseArray::ConstPtr &msg_poses);
};

}  // namespace occupy_planner

#endif  // SRC_TEB_LOCAL_PLANNER_SRC_OCCUPY_PLANNER_H_
