#include <actor_pos_publish_node.h>
using namespace std;
ActorPosPublishNode::ActorPosPublishNode()
{
	// Subscribers
	modelStates_sub = nh.subscribe("/gazebo/model_states", 1, &ActorPosPublishNode::modelStatesCallback, this);
	costmap_sub = nh.subscribe("/move_base/global_costmap/costmap",
							   1, &ActorPosPublishNode::costmapCallback, this);

	// Publishers
	actor1_target_pos_pub = nh.advertise<nav_msgs::Path>("/actor1/target", 100);
	actor2_target_pos_pub = nh.advertise<nav_msgs::Path>("/actor2/target", 100);
}

/****************************************FUNCTIONS***********************************************
************************************************************************************************/

// Get index from the string array by comparing a string
int getIndex(std::vector<std::string> v, std::string value)
{
	for (int i = 0; i < v.size(); i++)
	{
		if (v[i].compare(value) == 0)
			return i;
	}
	return -1;
}

// Takes a coordinate in map frame and transforms into idx corresponding in global costmap
int mapToCostmapIdx(double x, double y, int width, int height, double resolution)
{
	int a = (int)(y / resolution + (double)height / 2.0);
	int b = (int)(x / resolution + (double)width / 2.0);
	return a * width + b;
}

/****************************************CALLBACKS***********************************************
************************************************************************************************/

void ActorPosPublishNode::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg)
{
	costmap = *costmap_msg;
	height = costmap.info.height;
	width = costmap.info.width;
}

void ActorPosPublishNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states)
{
	int actor1_index = getIndex(model_states->name, "actor1");
	int actor2_index = getIndex(model_states->name, "actor2");

	// Set the position
	actor1_pose.pose.position = model_states->pose[actor1_index].position;
	actor2_pose.pose.position = model_states->pose[actor2_index].position;

	actor_message_arrived = true;
}

/****************************************MAIN LOOP***********************************************
************************************************************************************************/
void ActorPosPublishNode::mainLoop()
{
	// Check if the target position message is arrived
	if (actor_message_arrived)
	{
		// A star
		// geometry_msgs::Point start1(actor1_pose.pose.position.x, actor1_pose.pose.position.y);
		// geometry_msgs::Point start2(actor2_pose.pose.position.x, actor2_pose.pose.position.y);

		geometry_msgs::Point start1 = actor1_pose.pose.position;
		geometry_msgs::Point start2 = actor2_pose.pose.position;

		long unsigned n1 = planned_path1.poses.size();
		long unsigned n2 = planned_path2.poses.size();

		if (n1 < 10)
		{
			while (1)
			{
				goal1.x = rand() % width;
				goal1.y = rand() % height;

				if (costmap.data[goal1.x * width + (width - 1 - goal1.y)] == 0)
					break;
			}
		}
		planned_path1 = a_star(start1, goal1, costmap);

		if (n2 < 10)
		{
			while (1)
			{
				goal2.x = rand() % width;
				goal2.y = rand() % height;

				if (costmap.data[goal2.x * width + (width - 1 - goal2.y)] == 0)
					break;
			}
		}
		planned_path2 = a_star(start2, goal2, costmap);

		// Publish the target
		actor1_target_pos_pub.publish(planned_path1);
		actor2_target_pos_pub.publish(planned_path2);
		// std::cout << plan.x << " " << plan.y << std::endl;
	}
}
int main(int argc, char **argv)
{
	srand(time(0));
	ros::init(argc, argv, "actor_pos_publish");

	ActorPosPublishNode node;
	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		node.mainLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}