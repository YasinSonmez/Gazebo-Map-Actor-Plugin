#include <actor_pos_publish_node.h>
using namespace std;
ActorPosPublishNode::ActorPosPublishNode()
{
	// Subscribers
	modelStates_sub = nh.subscribe("/gazebo/model_states", 1, &ActorPosPublishNode::modelStatesCallback, this);
	costmap_sub = nh.subscribe("/move_base/global_costmap/costmap",
							   1, &ActorPosPublishNode::costmapCallback, this);
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

// Find the number of the actors in the scene (this is necessary for publishing purposes)
int getNumofActors(std::vector<std::string> v)
{
	int n = 0;
	for (int i = 0; i < v.size(); i++)
	{
		// Actor name convention must be {actor0, actor1, actor2 ...}
		if (v[i].compare(0, 5, "actor") == 0)
			n++;
	}
	return n;
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
	// If this is the first time we received model_states message then we
	// need to find the number of actors, then using push_back function we can
	// indicate the size of the vectors we will use
	if (!actor_message_arrived)
	{
		// Find the number of actors that we need to control, iterate over them
		num_of_actors = getNumofActors(model_states->name);
		for (int i = 0; i < num_of_actors; i++)
		{
			// Find the actor idxs provided by model_states corresponding to each actor
			actor_idxs.push_back(getIndex(model_states->name, "actor" + std::to_string(i)));
			// Find the position of each actor in map frame
			actor_poses.push_back(model_states->pose[actor_idxs[i]].position);
		}
		actor_message_arrived = true;
	}

	// Set the position of each actor pose if not the first time
	for (int i = 0; i < num_of_actors; i++)
		actor_poses[i] = model_states->pose[actor_idxs[i]].position;
}

/****************************************MAIN LOOP***********************************************
************************************************************************************************/
void ActorPosPublishNode::mainLoop()
{
	// Check if the target position message is arrived
	if (actor_message_arrived)
	{
		// Find the maximum possible distance from the actor we will generate targets to
		int max_len = (int)(7.0 / costmap.info.resolution);

		// Iterate over each actor to find the paths independently
		for (int i = 0; i < num_of_actors; i++)
		{
			// Find the index on the map corresponding to actor's position
			pair<int, int> init_idx = map_to_idx(
				actor_poses[i].x, actor_poses[i].y,
				costmap.info.width, costmap.info.height,
				costmap.info.origin.position.x, costmap.info.origin.position.y,
				costmap.info.resolution);

			// Determine whether to generate a new target. We will generate a new
			// target if it's the first time, there are less than 10 elements in
			// our path, or the generated path is over 200 elements(more computation)
			long unsigned n;
			if (actor_paths.size() == i)
				n = 0;
			else
				n = actor_paths[i].poses.size();

			if (n < 10 || n > 200)
			{
				// Iterate until a viable point is found
				while (1)
				{
					// Generate random targets close by the actor
					int x_idx = (init_idx.first - max_len / 2 + rand() % max_len) % width;
					int y_idx = (init_idx.second - max_len / 2 + rand() % max_len) % height;

					// Find how distant the point is from the actor, if less than max_len
					// and the costmap value corresponding to target is 0, proceed
					int difference = abs(init_idx.first - x_idx) + abs(init_idx.second - y_idx);
					if ((difference < max_len) && (costmap.data[x_idx * width + (width - 1 - y_idx)] == 0))
					{
						// If goals doesn't have enough values, push back. Otherwise, set the value
						if (goals.size() == i)
						{
							pair<int, int> temp;
							temp.first = x_idx;
							temp.second = y_idx;
							goals.push_back(temp);
						}
						else
						{
							goals[i].first = x_idx;
							goals[i].second = y_idx;
						}
						break;
					}
				}
			}
			// If actor_paths doesn't have enough values, push back. Otherwise, set the value
			if (actor_paths.size() == i)
				actor_paths.push_back(a_star(init_idx, goals[i], costmap));
			else
				actor_paths[i] = a_star(init_idx, goals[i], costmap);

			// If actor_target_pos_pubs doesn't have enough values, push back. Otherwise, publish
			if (actor_target_pos_pubs.size() == i)
				actor_target_pos_pubs.push_back(nh.advertise<nav_msgs::Path>("/actor" + std::to_string(i) + "/target", 10));
			// Publish the target
			else
				actor_target_pos_pubs[i].publish(actor_paths[i]);
		}
	}
}
int main(int argc, char **argv)
{
	srand(time(0));
	ros::init(argc, argv, "actor_pos_publish");

	ActorPosPublishNode node;
	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		node.mainLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}