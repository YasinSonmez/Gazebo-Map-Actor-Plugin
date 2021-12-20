#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

struct Cell
{
    int f, g, x, y;
    Cell() : f(0), g(0), x(0), y(0) {}
    Cell(int _f, int _g, int _x, int _y) : f(_f), g(_g), x(_x), y(_y) {}

    bool operator<(const Cell &rhs) const
    {
        return f > rhs.f;
    }
};
nav_msgs::Path a_star(geometry_msgs::Point init, geometry_msgs::Point goal, nav_msgs::OccupancyGrid costmapMsg);