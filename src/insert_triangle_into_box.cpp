#include "insert_triangle_into_box.h"

void insert_triangle_into_box(
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  BoundingBox & B)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // for each axis x, y, z
  for(int i=0; i<3; i++){
    double a_val = a[i];
    double b_val = b[i];
    double c_val = c[i];
    double b_min = B.min_corner[i];
    double b_max = B.max_corner[i];
    
    // extend B
    B.min_corner[i] = fmin(fmin(fmin(a_val, b_val), c_val), b_min);
    B.max_corner[i] = fmax(fmax(fmax(a_val, b_val), c_val), b_max);
  }
  
  ////////////////////////////////////////////////////////////////////////////
}


