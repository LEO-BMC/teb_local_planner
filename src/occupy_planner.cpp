#include "teb_local_planner/occupy_planner.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

namespace occupy_planner {

OccupyPlanner::OccupyPlanner(std::shared_ptr<ros::NodeHandle> nh_ptr) :
    nh_ptr_(std::move(nh_ptr)) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(*nh_ptr_);

  // setup dynamic reconfigure
  dynamic_recfg =
      boost::make_shared<dynamic_reconfigure::Server<teb_local_planner::TebLocalPlannerReconfigureConfig> >(*nh_ptr_);

  dynamic_recfg->setCallback(boost::bind(&OccupyPlanner::CB_reconfigure, this, _1, _2));

  // setup callback for custom obstacles
  custom_obst_sub = nh_ptr_->subscribe("obstacles", 1, &OccupyPlanner::CB_customObstacle, this);

  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = nh_ptr_->subscribe("/clicked_point", 5, &OccupyPlanner::CB_clicked_points, this);

  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = nh_ptr_->subscribe("via_points", 1, &OccupyPlanner::CB_via_points, this);

  sub_pose_stamped_start_ = std::make_shared<ros::Subscriber>(
      nh_ptr_->subscribe(
          "/occupy/pose_stamped_start",
          1,
          &OccupyPlanner::callback_pose_stamped_start,
          this));

  sub_pose_stamped_goal_ = std::make_shared<ros::Subscriber>(
      nh_ptr_->subscribe(
          "/occupy/pose_stamped_goal",
          1,
          &OccupyPlanner::callback_pose_stamped_goal,
          this));

  sub_ptr_poses_hybrid_astar_ = std::make_shared<ros::Subscriber>(
      nh_ptr_->subscribe(
          "/poses_hybrid_astar",
          1,
          &OccupyPlanner::callback_pose_stamped_goal,
          this));

  // interactive marker server for simulated dynamic obstacles
  marker_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("marker_obstacles");

  for (unsigned int i = 0; i < obst_vector.size(); ++i) {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/occupy_planner_exe/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(nh_ptr_->subscribe<geometry_msgs::Twist>(topic,
                                                                     1,
                                                                     boost::bind(&OccupyPlanner::CB_setObstacleVelocity,
                                                                                 this,
                                                                                 boost::placeholders::_1,
                                                                                 i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);
    // Add interactive markers for all point obstacles
    pobst = boost::dynamic_pointer_cast<teb_local_planner::PointObstacle>(obst_vector.at(i));
    if (pobst) {
      CreateInteractiveMarker(pobst->x(),
                              pobst->y(),
                              i,
                              config.map_frame,
                              &(*marker_server),
                              boost::bind(&OccupyPlanner::CB_obstacle_marker,
                                          this,
                                          boost::placeholders::_1));
    }
  }
  marker_server->applyChanges();

  // Setup visualization
  visual = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(*nh_ptr_, config));

  // Setup robot shape model
  robot_model = teb_local_planner::TebLocalPlannerROS::getRobotFootprintFromParamServer(*nh_ptr_);

  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::HomotopyClassPlanner(config,
                                                                                                 &obst_vector,
                                                                                                 robot_model,
                                                                                                 visual,
                                                                                                 &via_points));
  else
    planner = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(config,
                                                                                              &obst_vector,
                                                                                              robot_model,
                                                                                              visual,
                                                                                              &via_points));

  no_fixed_obstacles = obst_vector.size();

  cycle_timer = std::make_shared<ros::Timer>(
      nh_ptr_->createTimer(
          ros::Duration(0.025),
          &OccupyPlanner::CB_mainCycle,
          this));
  publish_timer = std::make_shared<ros::Timer>(
      nh_ptr_->createTimer(
          ros::Duration(0.1),
          &OccupyPlanner::CB_publishCycle,
          this));

}

// Planning loop
void OccupyPlanner::CB_mainCycle(const ros::TimerEvent &e) {
  if (msg_pose_stamped_start_ == nullptr || msg_pose_stamped_goal_ == nullptr) {
    ROS_WARN_STREAM("msg_pose_stamped_start_ or msg_pose_stamped_goal_ is nullptr");
    return;
  }
  Eigen::Affine3f affine;
  bool success = get_affine(affine, msg_pose_stamped_start_->header.frame_id, "base_link", ros::Time());
  if (!success) {
    ROS_WARN_STREAM("Couldn't obtain a tf in CB_mainCycle");
    return;
  }
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
  ROS_INFO_STREAM("did a plan");
}

// Visualization loop
void OccupyPlanner::CB_publishCycle(const ros::TimerEvent &e) {
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void OccupyPlanner::CB_reconfigure(teb_local_planner::TebLocalPlannerReconfigureConfig &reconfig, uint32_t level) {
  ROS_INFO_STREAM("CB_reconfigure is called.");
  config.reconfigure(reconfig);
  ROS_INFO_STREAM("CB_reconfigure ended.");
}

void OccupyPlanner::CreateInteractiveMarker(const double &init_x,
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

void OccupyPlanner::CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;

  if (index >= no_fixed_obstacles)
    return;
  teb_local_planner::PointObstacle
      *pobst = static_cast<teb_local_planner::PointObstacle *>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
}

void OccupyPlanner::CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg) {
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);

  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i) {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1) {
      if (obst_msg->obstacles.at(i).radius == 0) {
        obst_vector.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obst_msg->obstacles.at(
                                                                                                      i).polygon.points.front().x,
                                                                                                  obst_msg->obstacles.at(
                                                                                                      i).polygon.points.front().y)));
      } else {
        obst_vector.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::CircularObstacle(obst_msg->obstacles.at(
                                                                                                         i).polygon.points.front().x,
                                                                                                     obst_msg->obstacles.at(
                                                                                                         i).polygon.points.front().y,
                                                                                                     obst_msg->obstacles.at(
                                                                                                         i).radius)));
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

void OccupyPlanner::CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg) {
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back(Eigen::Vector2d(point_msg->point.x, point_msg->point.y));
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint <= 0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void OccupyPlanner::CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg) {
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses) {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void OccupyPlanner::CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr &twist_msg, const unsigned int id) {
  if (id >= obst_vector.size()) {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel(twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

void OccupyPlanner::callback_pose_stamped_start(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_start) {
  msg_pose_stamped_start_ = msg_pose_stamped_start;
}

void OccupyPlanner::callback_pose_stamped_goal(const geometry_msgs::PoseStamped::ConstPtr &msg_pose_stamped_goal) {
  msg_pose_stamped_goal_ = msg_pose_stamped_goal;
}

bool OccupyPlanner::get_affine(Eigen::Affine3f &affine,
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

}  // namespace occupy_planner