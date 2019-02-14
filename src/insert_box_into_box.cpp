#include "insert_box_into_box.h"

void insert_box_into_box(
  const BoundingBox & A,
  BoundingBox & B)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // for each axis x, y, z
  for(int i=0; i<3; i++){
    double a_min = A.min_corner[i];
    double b_min = B.min_corner[i];
    double a_max = A.max_corner[i];
    double b_max = B.max_corner[i];
    
    // extend B
    B.min_corner[i] = fmin(a_min, b_min);
    B.max_corner[i] = fmax(a_max, b_max);
  }
  
  ////////////////////////////////////////////////////////////////////////////
}

