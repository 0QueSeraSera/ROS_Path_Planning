#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");

  // Subscribe to map topic
  map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);

  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  // This loops until the node is running, will exit when the node is killed
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      ROS_INFO("init done. start planning");
      buildMapImage();
      drawGoalInitPose();
      plan();
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert mas to Point2D
  poseToPoint(init_pose_, msg->pose.pose); // init_pose_ 

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) { // isPointUnoccupied returns false
    init_pose_received_ = false;
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Initial pose obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Goal obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

Point2D RRTPlanner::randomPoint() {
  static std::default_random_engine generator;
  std::uniform_int_distribution<int> x_distribution(0, map_grid_->info.width - 1);
  std::uniform_int_distribution<int> y_distribution(0, map_grid_->info.height - 1);

  Point2D random_point;
  random_point.x(x_distribution(generator));
  random_point.y(y_distribution(generator));
  // ROS_INFO("randomPoint returns random_point: (%d, %d)", random_point.x(), random_point.y());
  return random_point;
}

int RRTPlanner::NearestNeighbour(const std::vector<treeNode> &tree, const Point2D &s) {
  int parent_id = -1;
  int w = map_grid_->info.width, h = map_grid_->info.height;
  double min_dist = w * w + h * h;
  int cur_x = s.x();
  int cur_y = s.y();
  for (int i = 0; i < tree.size(); i++) {
    treeNode node = tree[i];
    double cur_dist = calDist(node.pos, s);
    if (cur_dist < min_dist) {
      min_dist = cur_dist;
      parent_id = i;
    }
  }
  // ROS_INFO("NearestNeighbour returns parent_id: %d", parent_id);
  return parent_id;
}

void RRTPlanner::extractPath(const std::vector<treeNode> &tree, int goal_id) {

  // ROS_INFO("tree size: %d", tree.size());
  int cur_id = goal_id;
  while (cur_id != 0) {
    path_.push_back(tree[cur_id].pos);
    cur_id = tree[cur_id].parent_idx;
  }
  ROS_INFO("Path extracted");
  path_.push_back(tree[0].pos);
  std::reverse(path_.begin(), path_.end());
}

Point2D RRTPlanner::acquireNewState(double step_len, const Point2D &nearest, const Point2D &sample) {
  int dx = sample.x() - nearest.x();
  int dy = sample.y() - nearest.y();
  double angle = atan2(dy, dx);
  int new_x = nearest.x() + step_len * cos(angle);
  int new_y = nearest.y() + step_len * sin(angle);
  return Point2D(new_x, new_y);
}

void RRTPlanner::plan()
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;
  
  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
  double goal_threshold = 5; // distance threshold to goal
  double step_len = 5; // new state's distance to parent

  std::vector<treeNode> tree;
  treeNode start(init_pose_, 0); // init the start node with parent_id = 0
  tree.push_back(start);
  int maxIter = 100000;
  int iter = 0;
  while (not goal_received_) {
    iter++;
    Point2D new_sample = randomPoint();
    // ROS_INFO("new point: (%d, %d)", new_sample.x(), new_sample.y());
    if (isPointUnoccupied(new_sample)) {

      int parent_id = NearestNeighbour(tree, new_sample);
      // ROS_INFO("parent_id: %d, tree size: %d", parent_id, tree.size());
      int cur_id = tree.size();
      Point2D new_state = acquireNewState(step_len, tree[parent_id].pos, new_sample);
      if (not isPointUnoccupied(new_state)) {
        continue;
      }
      
      treeNode new_node(new_state, parent_id);

      tree.push_back(new_node);

      if (calDist(new_state, goal_) < goal_threshold) {
        ROS_INFO("goal reached with new point: (%d, %d)", new_state.x(), new_state.y());
        goal_received_ = true;

        extractPath(tree, cur_id);
      }
      if (iter > maxIter) {
        ROS_WARN("max iteration reached");
        break;
      }
    }
  }
  // visualize the tree here
  for (int i = 0; i < tree.size(); i++) {
    drawCircle(tree[i].pos, 2, cv::Scalar(0, 0, 255));
    if (i != 0) {
      drawLine(tree[i].pos, tree[tree[i].parent_idx].pos, cv::Scalar(0, 0, 255), 2);
    }
  }

  // Publish the calculated path
  publishPath();

}

void RRTPlanner::publishPath()
{
  // Create new Path msg
  nav_msgs::Path path;
  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT
  ROS_INFO("publishPath");
  for (int i = 0; i < path_.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = path_[i].x();
    pose.pose.position.y = path_[i].y();
    path.poses.push_back(pose);
  }
  // Publish the calculated path
  path_pub_.publish(path);

  drawPath();
  displayMapImage(10);
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p) // returns true if free
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map

  if (!map_received_)
  {
    ROS_ERROR("Map has not been received yet");
    return false;
  }

  // check if point is out of bounds
  if (p.x() < 0 || p.y() < 0 || p.x() >= map_grid_->info.width || p.y() >= map_grid_->info.height)
  {
    ROS_WARN("Point (%d, %d) is out of bounds", p.x(), p.y());
    return false;
  }

  // get occupancy value of point in the map
  int8_t occupancy = map_grid_->data[toIndex(p.x(), p.y())];

  // check if point is occupied
  if (occupancy > 0)
  {
    ROS_WARN("Point (%d, %d) is occupied", p.x(), p.y());
    return false;
  }

  return true;
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) { // (x * map_grid_->info.width + y;)
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::resizeWindow("Output", 600, 600);
  cv::waitKey(delay);
}


void RRTPlanner::drawPath(){
  for (int i = 0; i < path_.size() - 1; i++) {
    drawCircle(path_[i], 4, cv::Scalar(125, 125, 0));
    drawLine(path_[i], path_[i + 1], cv::Scalar(0, 255, 0), 4);
  }
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
// used for getting init_ and goal_ as Point2D objects
// the acquired points' origin is the bottom left corner of the map
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
  // p.x(pose.position.x / map_grid_->info.resolution);
  // p.y(pose.position.y / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y) //? conventionally, should it be y * width + height?
{
  return x * map_grid_->info.width + y;
}

inline double RRTPlanner::calDist(const Point2D & p1, const Point2D & p2)
{
  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
}

}  // namespace rrt_planner
