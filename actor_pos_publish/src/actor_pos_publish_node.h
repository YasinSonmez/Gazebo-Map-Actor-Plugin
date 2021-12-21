#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h> // tf to geometry_msgs conversion
#include "ros/topic.h"
#include <std_msgs/String.h>
#include <time.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "plan_utils.h"

using namespace std;

#define PI 3.14159265

class ActorPosPublishNode
{
private:
  // Callbacks
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states);
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg);

  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber modelStates_sub;
  ros::Subscriber costmap_sub;

  // Publishers
  std::vector<ros::Publisher> actor_target_pos_pubs;

  // Variables
  int height;
  int width;
  int num_of_actors = 0;

  std::vector<int> actor_idxs;
  std::vector<geometry_msgs::Point> actor_poses;
  std::vector<nav_msgs::Path> actor_paths;

  std::vector<pair<int, int>> goals;

  nav_msgs::OccupancyGrid costmap;
  bool actor_message_arrived = false;

  ros::Time begin;

public:
  ActorPosPublishNode();
  void mainLoop();
};