#include "point_box_squared_distance.h"

double point_box_squared_distance(
  const Eigen::RowVector3d & query,
  const BoundingBox & box)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // initialize
  double x_dist = 0;
  double y_dist = 0;
  double z_dist = 0;
  
  // out of bound
  if(query[0] > box.max_corner[0]){
    x_dist = query[0] - box.max_corner[0];
  }
  else if(query[0] < box.min_corner[0]){
    x_dist = box.min_corner[0] - query[0]; 
  }
  if(query[1] > box.max_corner[1]){
    y_dist = query[1] - box.max_corner[1];
  }
  else if(query[1] < box.min_corner[1]){
    y_dist = box.min_corner[1] - query[1]; 
  }
  if(query[2] > box.max_corner[2]){
    z_dist = query[2] - box.max_corner[2];
  }
  else if(query[2] < box.min_corner[2]){
    z_dist = box.min_corner[2] - query[2]; 
  }
  
  double distance = x_dist * x_dist + y_dist * y_dist + z_dist * z_dist;
  return distance;
  
  ////////////////////////////////////////////////////////////////////////////
}
