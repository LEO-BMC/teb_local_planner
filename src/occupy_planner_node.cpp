/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>


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
// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
teb_local_planner::PlannerInterfacePtr planner;
teb_local_planner::TebVisualizationPtr visual;
std::vector<teb_local_planner::ObstaclePtr> obst_vector;
teb_local_planner::ViaPointContainer via_points;
teb_local_planner::TebConfig config;
boost::shared_ptr<dynamic_reconfigure::Server<teb_local_planner::TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

bool get_affine(Eigen::Affine3f &affine,
                const std::string &frame_source,
                const std::string &frame_target,
                const ros::Time &time);

geometry_msgs::PoseStamped::ConstPtr msg_pose_stamped_start_;
geometry_msgs::PoseStamped::ConstPtr msg_pose_stamped_goal_;

void callback_pose_stamped_start(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_start);
void callback_pose_stamped_goal(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal);

std::shared_ptr<ros::Subscriber> sub_pose_stamped_start_;
std::shared_ptr<ros::Subscriber> sub_pose_stamped_goal_;
std::shared_ptr<ros::Subscriber> sub_ptr_poses_hybrid_astar_;

// =============== Main function =================
int main(int argc, char **argv) {
  ros::init(argc, argv, "occupy_planner_node");
  ros::NodeHandle n("~");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_pose_stamped_start_ = std::make_shared<ros::Subscriber>(
      n.subscribe(
          "/occupy/pose_stamped_start",
          1,
          callback_pose_stamped_start));

  sub_pose_stamped_goal_ = std::make_shared<ros::Subscriber>(
      n.subscribe(
          "/occupy/pose_stamped_goal",
          1,
          callback_pose_stamped_goal));

  sub_ptr_poses_hybrid_astar_ = std::make_shared<ros::Subscriber>(
      n.subscribe(
          "/poses_hybrid_astar",
          1,
          callback_pose_stamped_goal));

  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);

  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared<dynamic_reconfigure::Server<teb_local_planner::TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<teb_local_planner::TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);

  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);

  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);

  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);

  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

//  obst_vector.push_back(boost::make_shared<PointObstacle>(-3, 5));
//  obst_vector.push_back(boost::make_shared<PointObstacle>(6, 5));
//  obst_vector.push_back(boost::make_shared<PointObstacle>(0, 5));
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
//  Eigen::Vector2d vel(0, 0);
//  obst_vector.at(0)->setCentroidVelocity(vel);
//  vel = Eigen::Vector2d(0, 0);
//  obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */

  for (unsigned int i = 0; i < obst_vector.size(); ++i) {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/occupy_planner_exe/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);
    // Add interactive markers for all point obstacles
    boost::shared_ptr<teb_local_planner::PointObstacle> pobst = boost::dynamic_pointer_cast<teb_local_planner::PointObstacle>(obst_vector.at(i));
    if (pobst) {
      CreateInteractiveMarker(pobst->x(), pobst->y(), i, config.map_frame, &marker_server, &CB_obstacle_marker);
    }
  }
  marker_server.applyChanges();

  // Setup visualization
  visual = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(n, config));

  // Setup robot shape model
  teb_local_planner::RobotFootprintModelPtr robot_model = teb_local_planner::TebLocalPlannerROS::getRobotFootprintFromParamServer(n);

  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  else
    planner = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

  no_fixed_obstacles = obst_vector.size();
  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent &e) {
  if (msg_pose_stamped_start_ == nullptr || msg_pose_stamped_goal_ == nullptr)
    return;

  Eigen::Affine3f affine;
  bool success = get_affine(affine, msg_pose_stamped_start_->header.frame_id, "base_link", ros::Time());
  if (!success)
    return;

  auto pose_to_matrix = [](const geometry_msgs::Pose &pose) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    const auto &pos = pose.position;
    const auto &ori = pose.orientation;
    Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
    mat.topLeftCorner<3, 3>() = quat.toRotationMatrix();
    mat.topRightCorner<3, 1>() = Eigen::Vector3f(pos.x, pos.y, pos.z);
    return mat;
  };
  auto mat_start = pose_to_matrix(msg_pose_stamped_start_->pose);
  auto mat_goal = pose_to_matrix(msg_pose_stamped_goal_->pose);

  Eigen::Affine3f affine_trans_start;
  affine_trans_start.matrix() = affine * mat_start;
  Eigen::Affine3f affine_trans_goal;
  affine_trans_goal.matrix() = affine * mat_goal;

  auto affine_to_yaw = [](const Eigen::Affine3f &affine) {
    Eigen::Quaternionf quat(affine.rotation());
    tf2::Quaternion quat_tf(quat.x(), quat.y(), quat.z(), quat.w());
    return tf2::getYaw(quat_tf);
  };

  double yaw_start = affine_to_yaw(affine_trans_start);
  double yaw_goal = affine_to_yaw(affine_trans_goal);

  const auto &position_start = affine_trans_start.translation();
  const auto &position_goal = affine_trans_goal.translation();

  float extend_length = 1.0f;  // m
  float extend_x = extend_length * std::cos(yaw_start);
  float extend_y = extend_length * std::sin(yaw_start);

  planner->plan(
      teb_local_planner::PoseSE2(
          position_start.x() + extend_x,
          position_start.y() + extend_y,
          yaw_start),
          teb_local_planner::PoseSE2(
          position_goal.x(),
          position_goal.y(),
          yaw_goal));
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent &e) {
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(teb_local_planner::TebLocalPlannerReconfigureConfig &reconfig, uint32_t level) {
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double &init_x,
                             const double &init_y,
                             unsigned int id,
                             std::string frame,
                             interactive_markers::InteractiveMarkerServer *marker_server,
                             interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb) {
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  i_marker.controls.push_back(box_control);

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name, feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;

  if (index >= no_fixed_obstacles)
    return;
  teb_local_planner::PointObstacle *pobst = static_cast<teb_local_planner::PointObstacle *>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg) {
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);

  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i) {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1) {
      if (obst_msg->obstacles.at(i).radius == 0) {
        obst_vector.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y)));
      } else {
        obst_vector.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::CircularObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                               obst_msg->obstacles.at(i).polygon.points.front().y,
                                                               obst_msg->obstacles.at(i).radius)));
      }
    } else if (obst_msg->obstacles.at(i).polygon.points.empty()) {
      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
      continue;
    } else {
      teb_local_planner::PolygonObstacle *polyobst = new teb_local_planner::PolygonObstacle;
      for (size_t j = 0; j < obst_msg->obstacles.at(i).polygon.points.size(); ++j) {
        polyobst->pushBackVertex(obst_msg->obstacles.at(i).polygon.points[j].x,
                                 obst_msg->obstacles.at(i).polygon.points[j].y);
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(teb_local_planner::ObstaclePtr(polyobst));
    }

    if (!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities,
                                              obst_msg->obstacles.at(i).orientation);
  }
}

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg) {
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back(Eigen::Vector2d(point_msg->point.x, point_msg->point.y));
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint <= 0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg) {
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses) {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr &twist_msg, const unsigned int id) {
  if (id >= obst_vector.size()) {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel(twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

void callback_pose_stamped_start(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_start) {
  msg_pose_stamped_start_ = msg_pose_stamped_start;
}

void callback_pose_stamped_goal(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal) {
  msg_pose_stamped_goal_ = msg_pose_stamped_goal;
}

bool get_affine(Eigen::Affine3f &affine,
                const std::string &frame_source,
                const std::string &frame_target,
                const ros::Time &time) {

  auto get_transform_latest = [&](const std::string &source,
                                            const std::string &target)
      -> geometry_msgs::TransformStamped::Ptr {
    geometry_msgs::TransformStamped::Ptr transform_stamped = nullptr;
    try {
      transform_stamped.reset(new geometry_msgs::TransformStamped);

      *transform_stamped = tf_buffer_->lookupTransform(
          target,
          source,
          time);
    }
    catch (tf2::TransformException &ex) {
      transform_stamped = nullptr;
      ROS_WARN("%s", ex.what());
    }
    return transform_stamped;
  };

  geometry_msgs::TransformStamped::Ptr trans_init_ = get_transform_latest(
      frame_source,
      frame_target);
  if (trans_init_ == nullptr) {
    return false;
  }

  affine = tf2::transformToEigen(*trans_init_).cast<float>();

  return true;
}
