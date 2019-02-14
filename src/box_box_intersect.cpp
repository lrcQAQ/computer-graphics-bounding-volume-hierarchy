#include "box_box_intersect.h"
bool box_box_intersect(
  const BoundingBox & A,
  const BoundingBox & B)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // overlap on x, z, z axis
  bool x_cond = (B.min_corner[0] < A.max_corner[0]) && (A.min_corner[0] < B.max_corner[0]);
  bool y_cond = (B.min_corner[1] < A.max_corner[1]) && (A.min_corner[1] < B.max_corner[1]);
  bool z_cond = (B.min_corner[2] < A.max_corner[2]) && (A.min_corner[2] < B.max_corner[2]);
  
  bool res = x_cond && y_cond && z_cond;
  
  return res;
  
  ////////////////////////////////////////////////////////////////////////////
}

