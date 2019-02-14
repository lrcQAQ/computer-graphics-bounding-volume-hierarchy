#include "ray_intersect_box.h"
#include <iostream>

bool ray_intersect_box(
  const Ray & ray,
  const BoundingBox& box,
  const double min_t,
  const double max_t)
{
  ////////////////////////////////////////////////////////////////////////////

  // indices
  double x_e = ray.origin[0];
  double y_e = ray.origin[1];
  double z_e = ray.origin[2];
  double x_min = box.min_corner[0];
  double x_max = box.max_corner[0];
  double y_min = box.min_corner[1];
  double y_max = box.max_corner[1];
  double z_min = box.min_corner[2];
  double z_max = box.max_corner[2];
  
  // x
  double a = 1 / ray.direction[0];
  double tx_min, tx_max;
  if(a >= 0){
    tx_min = a * (x_min - x_e);
    tx_max = a * (x_max - x_e);
  }
  else{
    tx_min = a * (x_max - x_e);
    tx_max = a * (x_min - x_e);
  }
  
  // y 
  double b = 1 / ray.direction[1];
  double ty_min, ty_max;
  if(b >= 0){
    ty_min = b * (y_min - y_e);
    ty_max = b * (y_max - y_e);
  }
  else{
    ty_min = b * (y_max - y_e);
    ty_max = b * (y_min - y_e);
  }  
  
  // z 
  double c = 1 / ray.direction[2];
  double tz_min, tz_max;
  if(c >= 0){
    tz_min = c * (z_min - z_e);
    tz_max = c * (z_max - z_e);
  }
  else{
    tz_min = c * (z_max - z_e);
    tz_max = c * (z_min - z_e);
  }
  
  // check if have min > max violation
  double maximum_min = fmax(fmax(tx_min, ty_min), tz_min);
  double minimum_max = fmin(fmin(tx_max, ty_max), tz_max);
  if(maximum_min > minimum_max){
    return false;
  }
  
  // check outside [min_t, max_t] range
  if(maximum_min > max_t || minimum_max < min_t){
    return false;
  }
  
  return true;
  
  ////////////////////////////////////////////////////////////////////////////
}
