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
  
  node->param<std::string>("map_topic", map_topic, "/map");
  node->param<std::string>("path_topic", path_topic, "/path");
  //private_nh_.param<bool>("visualization", visualize_, true);
  node->param<double>("max_step", max_step_, 30.0);
  node->param<int>("max_vertices", max_vertices_, 2000);
  node->param<double>("goal_radius", goal_radius_, 15.0);

  ROS_INFO("Parameters received: max_step %f , max_vertices %d, goal_radius %f",max_step_,max_vertices_,goal_radius_);

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
  poseToPoint(init_pose_, msg->pose.pose);

  std::cout << "Coordinates are " << init_pose_.x() << " " << init_pose_.y() << std::endl;

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
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
  #if VIS > 0
  if (goal_received_) {
    drawCircle(goal_, 5, cv::Scalar(255, 20, 255));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 5, cv::Scalar(12, 255, 43));
  }
  #endif
}

void RRTPlanner::plan()
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;
  
  std::random_device rd;
  std::mt19937 eng(rd());

  std::uniform_int_distribution<int> distX(0, map_->cols - 1);
  std::uniform_int_distribution<int> distY(0, map_->rows - 1);

  Point2D q_random;

  graph_.reset(new Graph());
  graph_->add(init_pose_);

  int vertex_count = 0;
  found_goal_ = false;

  while(vertex_count < max_vertices_  && (!found_goal_)){    
    bool occupied = true;
    while (occupied) {
      q_random.x(distX(eng));
      q_random.y(distY(eng));
      if (isPointUnoccupied(q_random) && graph_->findNode(q_random) == nullptr)
        occupied = false;
    }
    Point2D q_near = findClosestPoint(q_random);
    auto newPoints = connectPoints(q_random,q_near);
    //std::cout << "New points printing" << std::endl;
    for(auto point : newPoints){
      //std::cout << point.x() <<" "<< point.y();
    }
    //std::cout << std::endl;
    addToGraph(newPoints,q_random);
    vertex_count++;  
  }
  
  if (found_goal_) {
    auto srcNode = graph_->findNode(init_pose_);
    auto destNode = graph_->findNode(goal_node_); 
    graph_->getPath(srcNode, destNode, path_);
    std::reverse(path_.begin(), path_.end());
    publishPath();
  } else{
    ROS_INFO("Could not reach goal");
  }
  std::cout << "No of vertex generated " << graph_->getSize() << std::endl;
  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
}

void RRTPlanner::publishPath()
{
  // Create new Path msg
  nav_msgs::Path path;
  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT

  // Publish the calculated path
  for(auto p : path_){
    path.poses.push_back(pointToPose(p));
  }

  #if VIS > 0
  drawCircle(init_pose_, 7 , cv::Scalar(0,0,255));
  for (int i = 0; i < path_.size() - 1; i++){
    drawLine(path_[i],path_[i+1],cv::Scalar(0,0,255),1);
  }
  drawCircle(goal_node_, 7 , cv::Scalar(0,0,255));
  #endif

  path_pub_.publish(path);

  displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map
  if(p.y() > map_grid_->info.height || p.x() > map_grid_->info.width){
    return false;
  }
  cv::Vec3b vec = map_->at<cv::Vec3b>( map_grid_->info.height - p.x() - 1, p.y());
  return (vec !=  cv::Vec3b(0,0,0));
}

Point2D RRTPlanner::findClosestPoint(Point2D & p){
  auto minDist = DBL_MAX;
  double dist;
  Point2D closest;

  for (auto &node : graph_->getNodes()) {

    auto root = node->getRoot();

    if (root == p)
      continue;

    dist = root.distance(p);

    if (dist < minDist) {
      minDist = dist;
      closest = root;
    }
  }
  return closest;
};

std::vector<Point2D> RRTPlanner::connectPoints(Point2D p1, Point2D p2){
  std::vector<Point2D> newPoints;
  newPoints.emplace_back(p2);

  cv::LineIterator linePoints(*map_, cv::Point(p2.x(), p2.y()), cv::Point(p1.x() , p1.y()),8);
  cv::Point point;
  Point2D q;
  bool blocked = false;
  

  for (int i = 0; i < linePoints.count; i++, linePoints++) {
    point = linePoints.pos();

    q.x(point.x);
    q.y(point.y);

    if (!isPointUnoccupied(q)) {
      blocked = true;
      break;
    }

    if (q.distance(newPoints[newPoints.size()-1]) > max_step_)
      newPoints.emplace_back(q);
  }

  if (!blocked) {
    if (newPoints[newPoints.size() - 1] != p1)
      newPoints.emplace_back(p1);
  }

  return newPoints;
};

void RRTPlanner::addToGraph(std::vector<Point2D>& newPoints,Point2D point){
  if (newPoints.size() > 1) {
    int p = 0;
    for (p = 0; p < newPoints.size() - 1; p++) {
      graph_->add(newPoints[p], newPoints[p + 1]);

      std::cout << newPoints[p+1].distance(goal_)  << std::endl;

      if(newPoints[p+1].distance(goal_) < goal_radius_){
        ROS_INFO("Ending search");  
        found_goal_=true;
        goal_node_=newPoints[p+1];
        break;
      }
      
      #if VIS > 0
        if (p != 0)
          drawCircle(newPoints[p], 3, cv::Scalar(0, 255, 255));
        drawLine(newPoints[p],newPoints[p+1], cv::Scalar(255,0,0),1);
      #endif
    }
    
    #if VIS > 0
    if (findInPoints(newPoints, point))
      drawCircle(point, 3, cv::Scalar(0, 255, 255));
    else
      drawCircle(newPoints[p+1], 3, cv::Scalar(0, 255, 255));
    #endif
  }
};

bool RRTPlanner::findInPoints(std::vector<Point2D> &points, Point2D point) {
  for (auto &p : points) {
    if (p == point)
      return true;
  }
  return false;
}

void RRTPlanner::buildMapImage()
{
  ROS_INFO("Building map!");
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
  int save = 1;
    if(save){
        cv::imwrite("/home/ubuntu/catkin_ws/src/navigation/rrt_planner_ros/resources/grey_map.bmp", *map_);
        ROS_INFO("Saved map!");
    }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(const Point2D & p, int radius, const cv::Scalar & color)
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
{
  p.x((pose.position.y / map_grid_->info.resolution));
  p.y((pose.position.x / map_grid_->info.resolution));
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}



}  // namespace rrt_planner
