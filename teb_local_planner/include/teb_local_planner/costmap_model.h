#ifndef COSTMAPMODEL_H
#define COSTMAPMODEL_H

#include <geometry_msgs/msg/point.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

namespace teb_local_planner
{
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

class CostmapModel
{
public:
    CostmapModel(nav2_costmap_2d::Costmap2D *costmap);

    double footprintCost(const geometry_msgs::msg::Point& position, const std::vector<geometry_msgs::msg::Point>& footprint,
                         double inscribed_radius, double circumscribed_radius);
    double lineCost(int x0, int x1, int y0, int y1) const;
    double pointCost(int x, int y) const;

    nav2_costmap_2d::Costmap2D *costmap_;
};

}
#endif // COSTMAPMODEL_H
