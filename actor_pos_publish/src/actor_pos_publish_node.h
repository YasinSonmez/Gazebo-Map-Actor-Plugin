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
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg);

  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber modelStates_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber costmap_sub;

  // Publishers
  ros::Publisher actor1_target_pos_pub;
  ros::Publisher actor2_target_pos_pub;

  // Variables
  int height;
  int width;
  geometry_msgs::Point goal1;
  geometry_msgs::Point goal2;
  nav_msgs::Path actor1_path;
  nav_msgs::Path actor2_path;
  geometry_msgs::PoseStamped actor1_pose;
  geometry_msgs::PoseStamped actor2_pose;
  nav_msgs::Path planned_path1;
  nav_msgs::Path planned_path2;

  nav_msgs::Odometry odom;
  nav_msgs::OccupancyGrid costmap;
  geometry_msgs::PoseStamped odom_in_map_frame;
  geometry_msgs::TransformStamped odom_to_map;
  tf2_ros::Buffer tf_buffer;
  bool actor_message_arrived = false;
  std::vector<geometry_msgs::PoseStamped> plan;

  ros::Time begin;

public:
  ActorPosPublishNode();
  void mainLoop();
};