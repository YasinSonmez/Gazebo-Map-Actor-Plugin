#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
using namespace std;
struct Cell
{
    int f, g, x, y;
    Cell() : f(0), g(0), x(0), y(0) {}
    Cell(int _f, int _g, int _x, int _y) : f(_f), g(_g), x(_x), y(_y) {}

    // This operator is used to sort the cells according to f values
    bool operator<(const Cell &rhs) const
    {
        return f > rhs.f;
    }
};
pair<int, int> map_to_idx(double x, double y, int width, int height, double originX, double originY, double resolution);
nav_msgs::Path a_star(pair<int, int> init, pair<int, int> goal, nav_msgs::OccupancyGrid costmapMsg);