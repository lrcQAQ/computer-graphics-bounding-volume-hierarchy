#include "nearest_neighbor_brute_force.h"
#include <limits>// std::numeric_limits<double>::infinity();
#include <iostream>

void nearest_neighbor_brute_force(
  const Eigen::MatrixXd & points,
  const Eigen::RowVector3d & query,
  int & I,
  double & sqrD)
{
  ////////////////////////////////////////////////////////////////////////////
  
  sqrD = std::numeric_limits<double>::infinity();
  
  // brute force all points
  for(int i=0; i<points.rows(); i++){
    // squared distance
    double curr_dist = (query - points.row(i)).squaredNorm();
    if(curr_dist < sqrD){
      I = i;
      sqrD = curr_dist;
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////
}
