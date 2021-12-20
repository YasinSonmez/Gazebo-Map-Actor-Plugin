#include "plan_utils.h"
using namespace std;

pair<double, double> idx_to_map(int x, int y, int width, int height, double originX, double originY, double resolution)
{
    double x_transformed = (-y - 0.5 + width) * resolution + originX;
    double y_transformed = (x + 0.5) * resolution + originY;
    return {x_transformed, y_transformed};
}

pair<int, int> map_to_idx(double x, double y, int width, int height, double originX, double originY, double resolution)
{
    int x_transformed = (y - originX) / resolution - 0.5;
    int y_transformed = -(x - originX) / resolution + width - 0.5;
    return {x_transformed, y_transformed};
}

nav_msgs::Path a_star(geometry_msgs::Point init, geometry_msgs::Point goal, nav_msgs::OccupancyGrid costmapMsg)
{
    double resolution = costmapMsg.info.resolution;
    int height = costmapMsg.info.height;
    int width = costmapMsg.info.width;

    double originX = costmapMsg.info.origin.position.x;
    double originY = costmapMsg.info.origin.position.y;

    pair<double, double> init_coords_in_idx = map_to_idx(init.x, init.y, width, height, originX, originY, resolution);
    // pair<double, double> goal_coords_in_idx = map_to_idx(goal.x, goal.y, width, height, originX, originY, resolution);
    pair<double, double> goal_coords_in_idx = {goal.x, goal.y};

    int delta[8][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}, {-1, 0}, {0, -1}, {1, 0}, {0, 1}};
    double delta_cost[8] = {1.414, 1.414, 1.414, 1.414, 1, 1, 1, 1};

    vector<vector<int>> costmap(height, vector<int>(width, 0));
    vector<vector<int>> heuristic(height, vector<int>(width, 0));

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            int currentData = costmapMsg.data[i * width + (width - 1 - j)];
            costmap[i][j] = currentData;
            heuristic[i][j] = abs(i - goal_coords_in_idx.first) + abs(j - goal_coords_in_idx.second);
        }

    vector<vector<int>> closed(height, vector<int>(width, 0));
    vector<vector<int>> action(height, vector<int>(width, 0));

    int x = init_coords_in_idx.first;
    int y = init_coords_in_idx.second;
    double g = costmap[x][y];
    double f = g + heuristic[x][y];
    priority_queue<Cell> cells;

    cells.push(Cell(f, g, x, y));
    bool found = false;
    bool resign = false;
    while (!found && !resign)
    {
        if (cells.size() == 0)
        {
            nav_msgs::Path empty_path;
            return empty_path;
        }

        Cell next = cells.top();
        cells.pop();
        int x = next.x;
        int y = next.y;
        double g = next.g;

        if (x == goal_coords_in_idx.first and y == goal_coords_in_idx.second)
        {
            found = true;

            geometry_msgs::PoseStamped tmp_pose;
            nav_msgs::Path my_path;
            while (x != init_coords_in_idx.first || y != init_coords_in_idx.second)
            {
                int move_idx = action[x][y];
                x -= delta[move_idx][0];
                y -= delta[move_idx][1];
                pair<double, double> coords_in_map = idx_to_map(x, y, width, height, originX, originY, resolution);

                tmp_pose.pose.position.x = coords_in_map.first;
                tmp_pose.pose.position.y = coords_in_map.second;
                // Header info
                tmp_pose.header.stamp = ros::Time::now();
                tmp_pose.header.frame_id = "map";

                my_path.poses.push_back(tmp_pose);
            }
            // Header info
            my_path.header.stamp = ros::Time::now();
            my_path.header.frame_id = "map";
            pair<double, double> coords_in_map = idx_to_map(x, y, width, height, originX, originY, resolution);

            return my_path;
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                int x2 = x + delta[i][0];
                int y2 = y + delta[i][1];
                if (x2 >= 0 && x2 < height and y2 >= 0 and y2 < width)
                {
                    if (closed[x2][y2] == 0)
                    {
                        double g2 = g + costmap[x2][y2] + 0.1 * delta_cost[i];
                        // int g2 = g + costmap[x2][y2] + 3;
                        double f2 = g2 + heuristic[x2][y2];
                        cells.push(Cell(f2, g2, x2, y2));
                        closed[x2][y2] = 1;
                        action[x2][y2] = i;
                    }
                }
            }
        }
    }
    nav_msgs::Path empty_path;
    return empty_path;
}