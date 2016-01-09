/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/


#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <complex>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <geometry_msgs/Polygon.h>


namespace teb_local_planner
{

/**
 * @class Obstacle
 * @brief Abstract class that defines the interface for modelling obstacles
 */
class Obstacle
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    */
  Obstacle() : dynamic_(false), centroid_velocity_(Eigen::Vector2d::Zero())
  {
  }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~Obstacle()
  {
  }


  /** @name Centroid coordinates (abstract, obstacle type depending) */
  //@{ 

  /**
    * @brief Get centroid coordinates of the obstacle
    * @return Eigen::Vector2d containing the centroid
    */
  virtual const Eigen::Vector2d& getCentroid() const = 0;

  /**
    * @brief Get centroid coordinates of the obstacle as complex number
    * @return std::complex containing the centroid coordinate
    */
  virtual std::complex<double> getCentroidCplx() const = 0;

  //@}


  /** @name Collision checking and distance calculations (abstract, obstacle type depending) */
  //@{ 

  /**
    * @brief Check if a given point collides with the obstacle
    * @param position 2D reference position that should be checked
    * @param min_dist Minimum distance allowed to the obstacle to be collision free
    * @return \c true if position is inside the region of the obstacle or if the minimum distance is lower than min_dist
    */
  virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const = 0;

  /**
    * @brief Check if a given line segment between two points intersects with the obstacle (and additionally keeps a safty distance \c min_dist)
    * @param line_start 2D point for the end of the reference line
    * @param line_end 2D point for the end of the reference line
    * @param min_dist Minimum distance allowed to the obstacle to be collision/intersection free
    * @return \c true if given line intersects the region of the obstacle or if the minimum distance is lower than min_dist
    */
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const = 0;

  /**
    * @brief Get the minimum euclidean distance to the obstacle
    * @return The nearest possible distance to the obstacle
    */
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const = 0;


  /**
   * @brief Get the closest point on the boundary of the obstacle w.r.t. a specified reference position
   * @param position reference 2d position
   * @return closest point on the obstacle boundary
   */
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const = 0;

  //@}



  /** @name Velocity related methods for non-static, moving obstacles */
  //@{ 

  /**
    * @brief Check if the obstacle is a moving with a (non-zero) velocity
    * @return \c true if the obstacle is not marked as static, \c false otherwise
    */	
  bool isDynamic() const {return dynamic_;}

  /**
    * @brief Set the 2d velocity (vx, vy) of the obstacle w.r.t to the centroid
    * @remarks Setting the velocity using this function marks the obstacle as dynamic (@see isDynamic)
    * @param vel 2D vector containing the velocities of the centroid in x and y directions
    */
  void setCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d>& vel) {centroid_velocity_ = vel; dynamic_=true;} 

  /**
    * @brief Get the obstacle velocity (vx, vy) (w.r.t. to the centroid)
    * @returns 2D vector containing the velocities of the centroid in x and y directions
    */
  const Eigen::Vector2d& getCentroidVelocity() const {return centroid_velocity_;}

  //@}



  /** @name Helper Functions */
  //@{ 

  /**
    * @brief Helper function to calculate the distance between a line segment and a point
    * @param point 2D point
    * @param line_start 2D point representing the start of the line segment
    * @param line_end 2D point representing the end of the line segment
    * @return minimum distance to a given line segment
    */
  static double DistanceFromLineSegment(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end);

  /**
    * @brief Helper function to obtain the closest point on a line segment w.r.t. a reference point
    * @param point 2D point
    * @param line_start 2D point representing the start of the line segment
    * @param line_end 2D point representing the end of the line segment
    * @return Closest point on the line segment
    */
  static Eigen::Vector2d ClosestPointOnLineSegment(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end);

  /**
    * @brief Helper function to check whether two line segments intersects
    * @param line1_start 2D point representing the start of the first line segment
    * @param line1_end 2D point representing the end of the first line segment
    * @param line2_start 2D point representing the start of the second line segment
    * @param line2_end 2D point representing the end of the second line segment
    * @param[out] intersection [optional] Write intersection point to destination (the value is only written, if both lines intersect, e.g. if the function returns \c true)
    * @return \c true if both line segments intersect
    */	
  static bool CheckLineSegmentsIntersection(const Eigen::Ref<const Eigen::Vector2d>& line1_start, const Eigen::Ref<const Eigen::Vector2d>& line1_end, 
						    const Eigen::Ref<const Eigen::Vector2d>& line2_start, const Eigen::Ref<const Eigen::Vector2d>& line2_end, Eigen::Vector2d* intersection = NULL);
  
    /**
    * @brief Helper function to calculate the smallest distance between two line segments
    * @param line1_start 2D point representing the start of the first line segment
    * @param line1_end 2D point representing the end of the first line segment
    * @param line2_start 2D point representing the start of the second line segment
    * @param line2_end 2D point representing the end of the second line segment
    * @return smallest distance between both segments
    */  
  static double DistanceSegmentToSegment2d(const Eigen::Ref<const Eigen::Vector2d>& line1_start, const Eigen::Ref<const Eigen::Vector2d>& line1_end, 
                  const Eigen::Ref<const Eigen::Vector2d>& line2_start, const Eigen::Ref<const Eigen::Vector2d>& line2_end);

  
  /**
   * @brief Convert the obstacle to a polygon message
   * 
   * Convert the obstacle to a corresponding polygon msg.
   * Point obstacles have one vertex, lines have two vertices 
   * and polygons might are implictly closed such that the start vertex must not be repeated.
   * @param[out] polygon the polygon message
   */
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon) = 0;

  //@}
	
protected:
	   
  bool dynamic_; //!< Store flag if obstacle is dynamic (resp. a moving obstacle)
  Eigen::Vector2d centroid_velocity_; //!< Store the corresponding velocity (vx, vy) of the centroid (zero, if _dynamic is \c true)
  
public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<Obstacle> ObstaclePtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const Obstacle> ObstacleConstPtr;
//! Abbrev. for containers storing multiple obstacles
typedef std::vector<ObstaclePtr> ObstContainer;



/**
 * @class PointObstacle
 * @brief Implements a 2D point obstacle
 */
class PointObstacle : public Obstacle
{
public:
  
  /**
    * @brief Default constructor of the point obstacle class
    */
  PointObstacle() : Obstacle(), pos_(Eigen::Vector2d::Zero())
  {}
  
  /**
    * @brief Construct PointObstacle using a 2d position vector
    * @param position 2d position that defines the current obstacle position
    */
  PointObstacle(const Eigen::Ref< const Eigen::Vector2d>& position) : Obstacle(), pos_(position)
  {}
  
  /**
    * @brief Construct PointObstacle using x- and y-coordinates
    * @param x x-coordinate
    * @param y y-coordinate
    */      
  PointObstacle(double x, double y) : Obstacle(), pos_(Eigen::Vector2d(x,y))
  {}


  // implements checkCollision() of the base class
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
  {
      return getMinimumDistance(point) < min_dist;
  }
  
  
  // implements checkLineIntersection() of the base class
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const
  {   
      // Distance Line - Circle
      // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
      Eigen::Vector2d a = line_end-line_start; // not normalized!  a=y-x
      Eigen::Vector2d b = pos_-line_start; // b=m-x
      
      // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
      double t = a.dot(b)/a.dot(a);
      if (t<0) t=0; // bound t (since a is not normalized, t can be scaled between 0 and 1 to parametrize the line
      else if (t>1) t=1;
      Eigen::Vector2d nearest_point = line_start + a*t;
      
      // check collision
      return checkCollision(nearest_point, min_dist);
  }

  
  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const
  {
    return (position-pos_).norm();
  }
    
  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
  {
    return pos_;
  }
  
  // implements getCentroid() of the base class
  virtual const Eigen::Vector2d& getCentroid() const
  {
    return pos_;
  }
  
  
  // implements getCentroidCplx() of the base class
  virtual std::complex<double> getCentroidCplx() const
  {
    return std::complex<double>(pos_[0],pos_[1]);
  }
  
  // Accessor methods
  const Eigen::Vector2d& position() const {return pos_;} //!< Return the current position of the obstacle (read-only)
  Eigen::Vector2d& position() {return pos_;} //!< Return the current position of the obstacle
  double& x() {return pos_.coeffRef(0);} //!< Return the current x-coordinate of the obstacle
  const double& x() const {return pos_.coeffRef(0);} //!< Return the current y-coordinate of the obstacle (read-only)
  double& y() {return pos_.coeffRef(1);} //!< Return the current x-coordinate of the obstacle
  const double& y() const {return pos_.coeffRef(1);} //!< Return the current y-coordinate of the obstacle (read-only)
      
  // implements toPolygonMsg() of the base class
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon)
  {
    polygon.points.resize(1);
    polygon.points.front().x = pos_.x();
    polygon.points.front().y = pos_.y();
    polygon.points.front().z = 0;
  }
      
protected:
  
  Eigen::Vector2d pos_; //!< Store the position of the PointObstacle
  
  	
public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};



/**
* @class LineObstacle
* @brief Implements a 2D line obstacle
*/
  
class LineObstacle : public Obstacle
{
public:
  //! Abbrev. for a container storing vertices (2d points defining the edge points of the polygon)
  typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > VertexContainer;
  
  /**
    * @brief Default constructor of the point obstacle class
    */
  LineObstacle() : Obstacle()
  {
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
  }
  
  /**
   * @brief Construct LineObstacle using 2d position vectors as start and end of the line
   * @param line_start 2d position that defines the start of the line obstacle
   * @param line_end 2d position that defines the end of the line obstacle
   */
  LineObstacle(const Eigen::Ref< const Eigen::Vector2d>& line_start, const Eigen::Ref< const Eigen::Vector2d>& line_end) 
                : Obstacle(), start_(line_start), end_(line_end)
  {
    calcCentroid();
  }
  
  /**
   * @brief Construct LineObstacle using start and end coordinates
   * @param x1 x-coordinate of the start of the line
   * @param y1 y-coordinate of the start of the line
   * @param x2 x-coordinate of the end of the line
   * @param y2 y-coordinate of the end of the line
   */
  LineObstacle(double x1, double y1, double x2, double y2) : Obstacle()     
  {
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    calcCentroid();
  }

  // implements checkCollision() of the base class
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const    
  {
    return getMinimumDistance(point) <= min_dist;
  }
  
  // implements checkLineIntersection() of the base class
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const 
  {
    return CheckLineSegmentsIntersection(line_start, line_end, start_, end_);
  }

  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const 
  {
    return DistanceFromLineSegment(position, start_, end_);
  }

  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
  {
    return ClosestPointOnLineSegment(position, start_, end_);
  }


  
  // implements getCentroid() of the base class
  virtual const Eigen::Vector2d& getCentroid() const    
  {
    return centroid_;
  }
  
  // implements getCentroidCplx() of the base class
  virtual std::complex<double> getCentroidCplx() const  
  {
    return std::complex<double>(centroid_.x(), centroid_.y());
  }
  
  // Access or modify line
  const Eigen::Vector2d& start() const {return start_;}
  void setStart(const Eigen::Ref<const Eigen::Vector2d>& start) {start_ = start; calcCentroid();}
  const Eigen::Vector2d& end() const {return end_;}
  void setEnd(const Eigen::Ref<const Eigen::Vector2d>& end) {end_ = end; calcCentroid();}
  
  // implements toPolygonMsg() of the base class
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon)
  {
    polygon.points.resize(2);
    polygon.points.front().x = start_.x();
    polygon.points.front().y = start_.y();
    
    polygon.points.back().x = end_.x();
    polygon.points.back().y = end_.y();
    polygon.points.back().z = polygon.points.front().z = 0;
  }
  
protected:
  void calcCentroid()	{	centroid_ = 0.5*(start_ + end_); }
  
private:
	Eigen::Vector2d start_;
	Eigen::Vector2d end_;
	
  Eigen::Vector2d centroid_;

public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};
  

/**
 * @class PolygonObstacle
 * @brief Implements a polygon obstacle with an arbitrary number of vertices
 * @details If the polygon has only 2 vertices, than it is considered as a line,
 * 	    otherwise the polygon will always be closed (a connection between the first and the last vertex
 * 	    is included automatically).
 */
class PolygonObstacle : public Obstacle
{
public:
  
  //! Abbrev. for a container storing vertices (2d points defining the edge points of the polygon)
  typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > VertexContainer;
  
  /**
    * @brief Default constructor of the polygon obstacle class
    */
  PolygonObstacle() : Obstacle(), finalized_(false)
  {
    centroid_.setConstant(NAN);
  }
  
  
  /* FIXME Not working at the moment due to the aligned allocator version of std::vector
    * And it is C++11 code that is disabled atm to ensure compliance with ROS indigo/jade
  template <typename... Vector2dType>
  PolygonObstacle(const Vector2dType&... vertices) : _vertices({vertices...})
  { 
    calcCentroid();
    _finalized = true;
  }
  */

  
  // implements checkCollision() of the base class
  virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
  {
      // line case
      if (noVertices()==2)
        return getMinimumDistance(point) <= min_dist;
    
      // check if point is in the interior of the polygon
      // point in polygon test - raycasting (http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html)
      // using the following algorithm we may obtain false negatives on edge-cases, but that's ok for our purposes	
      size_t i, j;
      bool c = false;
      for (i = 0, j = noVertices()-1; i < noVertices(); j = i++) 
      {
        if ( ((vertices_.at(i).y()>point.y()) != (vertices_.at(j).y()>point.y())) &&
              (point.x() < (vertices_.at(j).x()-vertices_.at(i).x()) * (point.y()-vertices_.at(i).y()) / (vertices_.at(j).y()-vertices_.at(i).y()) + vertices_.at(i).x()) )
            c = !c;
      }
      if (c>0) return true;

      // If this statement is reached, the point lies outside the polygon or maybe on its edges
      // Let us check the minium distance as well
      return min_dist == 0 ? false : getMinimumDistance(point) < min_dist;
  }
  

  /**
    * @brief Check if a given line segment between two points intersects with the obstacle (and additionally keeps a safty distance \c min_dist)
    * @param line_start 2D point for the end of the reference line
    * @param line_end 2D point for the end of the reference line
    * @param min_dist Minimum distance allowed to the obstacle to be collision/intersection free
    * @remarks we ignore \c min_dist here
    * @return \c true if given line intersects the region of the obstacle or if the minimum distance is lower than min_dist
    */
  virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const;


  // implements getMinimumDistance() of the base class
  virtual double getMinimumDistance(const Eigen::Vector2d& position) const;
  
  // implements getMinimumDistanceVec() of the base class
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const;
  
  // implements getCentroid() of the base class
  virtual const Eigen::Vector2d& getCentroid() const
  {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return centroid_;
  }
  
  // implements getCentroidCplx() of the base class
  virtual std::complex<double> getCentroidCplx() const
  {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return std::complex<double>(centroid_.coeffRef(0), centroid_.coeffRef(1));
  }
  
  // implements toPolygonMsg() of the base class
  virtual void toPolygonMsg(geometry_msgs::Polygon& polygon);
  
  
  /** @name Define the polygon */
  ///@{       
  
  // Access or modify polygon
  const VertexContainer& vertices() const {return vertices_;} //!< Access vertices container (read-only)
  VertexContainer& vertices() {return vertices_;} //!< Access vertices container
  
  /**
    * @brief Add a vertex to the polygon (edge-point)
    * @remarks You do not need to close the polygon (do not repeat the first vertex)
    * @warning Do not forget to call finalizePolygon() after adding all vertices
    * @param vertex 2D point defining a new polygon edge
    */
  void pushBackVertex(const Eigen::Ref<const Eigen::Vector2d>& vertex)
  {
    vertices_.push_back(vertex);
    finalized_ = false;
  }
  
  /**
    * @brief Add a vertex to the polygon (edge-point)
    * @remarks You do not need to close the polygon (do not repeat the first vertex)
    * @warning Do not forget to call finalizePolygon() after adding all vertices
    * @param x x-coordinate of the new vertex
    * @param y y-coordinate of the new vertex
    */  
  void pushBackVertex(double x, double y)
  {
    vertices_.push_back(Eigen::Vector2d(x,y));
    finalized_ = false;
  }
  
  /**
    * @brief Call finalizePolygon after the polygon is created with the help of pushBackVertex() methods
    */
  void finalizePolygon()
  {
    fixPolygonClosure();
    calcCentroid();
    finalized_ = true;
  }
  
  /**
    * @brief Clear all vertices (Afterwards the polygon is not valid anymore)
    */
  void clearVertices() {vertices_.clear(); finalized_ = false;}
  
  /**
    * @brief Get the number of vertices defining the polygon (the first vertex is counted once)
    */
  size_t noVertices() const {return vertices_.size();}
  
  
  ///@}
      
protected:
  
  void fixPolygonClosure(); //!< Check if the current polygon contains the first vertex twice (as start and end) and in that case erase the last redundant one.

  void calcCentroid(); //!< Compute the centroid of the polygon (called inside finalizePolygon())

  
  VertexContainer vertices_; //!< Store vertices defining the polygon (@see pushBackVertex)
  Eigen::Vector2d centroid_; //!< Store the centroid coordinates of the polygon (@see calcCentroid)
  
  bool finalized_; //!< Flat that keeps track if the polygon was finalized after adding all vertices

  
  	
public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};


} // namespace teb_local_planner

#endif /* OBSTACLES_H */
