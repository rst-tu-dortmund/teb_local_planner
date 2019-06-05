#include <dwb_critics/line_iterator.hpp>

#include "teb_local_planner/costmap_model.h"

namespace teb_local_planner
{
CostmapModel::CostmapModel(nav2_costmap_2d::Costmap2D *costmap) : costmap_(costmap)
{

}
double CostmapModel::footprintCost(const geometry_msgs::msg::Point& position, const std::vector<geometry_msgs::msg::Point>& footprint,
                            double inscribed_radius, double circumscribed_radius){

    //used to put things into grid coordinates
    unsigned int cell_x, cell_y;

    //get the cell coord of the center point of the robot
    if(!costmap_->worldToMap(position.x, position.y, cell_x, cell_y))
        return -1.0;

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    if(footprint.size() < 3){
        unsigned char cost = costmap_->getCost(cell_x, cell_y);
        //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
        if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION)
            return -1.0;
        return cost;
    }

    //now we really have to lay down the footprint in the costmap grid
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    //we need to rasterize each line in the footprint
    for(unsigned int i = 0; i < footprint.size() - 1; ++i){
        //get the cell coord of the first point
        if(!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0))
            return -1.0;

        //get the cell coord of the second point
        if(!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
            return -1.0;

        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = std::max(line_cost, footprint_cost);

        //if there is an obstacle that hits the line... we know that we can return false right away
        if(line_cost < 0)
            return -1.0;
    }

    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    if(!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0))
        return -1.0;

    //get the cell coord of the first point
    if(!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1))
        return -1.0;

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if(line_cost < 0)
        return -1.0;

    //if all line costs are legal... then we can return that the footprint is legal
    return footprint_cost;
}
double CostmapModel::lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for( dwb_critics::LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
    {
        point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

        if(point_cost < 0)
            return -1;

        if(line_cost < point_cost)
            line_cost = point_cost;
    }

    return line_cost;
}

double CostmapModel::pointCost(int x, int y) const {
    unsigned char cost = costmap_->getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    //if(cost == LETHAL_OBSTACLE){
    if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION){
        return -1;
    }

    return cost;
}
}
