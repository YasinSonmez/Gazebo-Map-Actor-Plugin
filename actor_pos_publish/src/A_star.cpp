#include "A_star.h"
using namespace std;

// Convert a map index to corresponding coordinate
pair<double, double> idx_to_map(int x, int y, int width, int height, double originX, double originY, double resolution)
{
    double x_transformed = (-y - 0.5 + width) * resolution + originX;
    double y_transformed = (x + 0.5) * resolution + originY;
    return {x_transformed, y_transformed};
}
// Convert a coordinate to corresponding map index
pair<int, int> map_to_idx(double x, double y, int width, int height, double originX, double originY, double resolution)
{
    int x_transformed = (y - originX) / resolution - 0.5;
    int y_transformed = -(x - originX) / resolution + width - 0.5;
    return {x_transformed, y_transformed};
}

nav_msgs::Path a_star(pair<int, int> init, pair<int, int> goal, nav_msgs::OccupancyGrid costmapMsg)
{
    double resolution = costmapMsg.info.resolution;
    int height = costmapMsg.info.height;
    int width = costmapMsg.info.width;

    double originX = costmapMsg.info.origin.position.x;
    double originY = costmapMsg.info.origin.position.y;

    // Delta is the possible movements in the graph, there are 8 neighbors of each cell
    int delta[8][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}, {-1, 0}, {0, -1}, {1, 0}, {0, 1}};
    // Delta cost is the movement cost along that direction
    double delta_cost[8] = {1.414, 1.414, 1.414, 1.414, 1, 1, 1, 1};

    vector<vector<int>> costmap(height, vector<int>(width, 0));
    vector<vector<int>> heuristic(height, vector<int>(width, 0));

    // Turn the 1D costmap into 2D costmap and calculate the heuristic
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            int currentData = costmapMsg.data[i * width + (width - 1 - j)];
            costmap[i][j] = currentData;
            heuristic[i][j] = abs(i - goal.first) + abs(j - goal.second);
        }

    // Closed is the cells that are already visited
    vector<vector<int>> closed(height, vector<int>(width, 0));
    // Action is the direction corresponding to best move in that cell
    vector<vector<int>> action(height, vector<int>(width, 0));

    int x = init.first;
    int y = init.second;
    double g = costmap[x][y];
    double f = g + heuristic[x][y];
    // Create a priority cell to store the values in order. There is the operator
    // to rank all the cells in the header file
    priority_queue<Cell> cells;
    cells.push(Cell(f, g, x, y));

    bool found = false;
    bool resign = false;
    while (!found && !resign)
    {
        // If there are no cells return empty path
        if (cells.size() == 0)
        {
            nav_msgs::Path empty_path;
            return empty_path;
        }

        // Get the best next cell according to f value
        Cell next = cells.top();
        cells.pop();
        int x = next.x;
        int y = next.y;
        double g = next.g;

        // Check if the goal is reached
        if (x == goal.first and y == goal.second)
        {
            found = true;

            geometry_msgs::PoseStamped tmp_pose;
            nav_msgs::Path my_path;
            // Iterate until the initil point is reached. Move back from goal
            // to init, following the best move recorded in the action
            while (x != init.first || y != init.second)
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
        // If the cell is not the goal position find its childs and add to the pool
        else
        {
            // Iterate over all possible actions
            for (int i = 0; i < 8; i++)
            {
                int x2 = x + delta[i][0];
                int y2 = y + delta[i][1];
                // Check to stay in bounds of the map
                if (x2 >= 0 && x2 < height and y2 >= 0 and y2 < width)
                {
                    // If the cell is not visited already select it
                    if (closed[x2][y2] == 0)
                    {
                        // Calculate the cost to reach that state from the parent
                        double g2 = g + costmap[x2][y2] + 0.1 * delta_cost[i];
                        // Calculate the f value, sum of g and the heuristic
                        double f2 = g2 + heuristic[x2][y2];
                        // Push the cell to cells queue and sort accordingly
                        cells.push(Cell(f2, g2, x2, y2));
                        // Indicate that this state is visited
                        closed[x2][y2] = 1;
                        // Record the best action
                        action[x2][y2] = i;
                    }
                }
            }
        }
    }
    nav_msgs::Path empty_path;
    return empty_path;
}