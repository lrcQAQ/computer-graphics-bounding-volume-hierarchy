#include "triangle_triangle_intersection.h"


#include "Ray.h"
#include "ray_intersect_triangle.h"


bool triangle_triangle_intersection(
  const Eigen::RowVector3d & A0,
  const Eigen::RowVector3d & A1,
  const Eigen::RowVector3d & A2,
  const Eigen::RowVector3d & B0,
  const Eigen::RowVector3d & B1,
  const Eigen::RowVector3d & B2)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // reference: https://github.com/alecjacobson/computer-graphics-bounding-volume-hierarchy/issues/33
  
  // range of parametric distance limit
  double min_t = 0;
  double max_t = 1;
  
  // using A's edges as ray: A0A1, A1A2, A2A0
  // check if ray hits triangle B
  Eigen::Vector3d a1_origin = A0;
  Eigen::Vector3d a1_direction = A1 - A0;
  Ray a1(a1_origin, a1_direction);
  Eigen::Vector3d a2_origin = A1;
  Eigen::Vector3d a2_direction = A2 - A1;
  Ray a2(a2_origin, a2_direction);
  Eigen::Vector3d a3_origin = A2;
  Eigen::Vector3d a3_direction = A0 - A2;
  Ray a3(a3_origin, a3_direction);
  
  double t;
  bool a1_hit = ray_intersect_triangle(a1, B0, B1, B2, min_t, max_t, t);
  bool a2_hit = ray_intersect_triangle(a2, B0, B1, B2, min_t, max_t, t);
  bool a3_hit = ray_intersect_triangle(a3, B0, B1, B2, min_t, max_t, t);
  
  // using B's edges as ray: B0B1, B1B2, B2B0
  // check if ray hits triangle A
  Eigen::Vector3d b1_origin = B0;
  Eigen::Vector3d b1_direction = B1 - B0;
  Ray b1(b1_origin, b1_direction);
  Eigen::Vector3d b2_origin = B1;
  Eigen::Vector3d b2_direction = B2 - B1;
  Ray b2(b2_origin, b2_direction);
  Eigen::Vector3d b3_origin = B2;
  Eigen::Vector3d b3_direction = B0 - B2;
  Ray b3(b3_origin, b3_direction);
  
  bool b1_hit = ray_intersect_triangle(b1, A0, A1, A2, min_t, max_t, t);
  bool b2_hit = ray_intersect_triangle(b2, A0, A1, A2, min_t, max_t, t);
  bool b3_hit = ray_intersect_triangle(b3, A0, A1, A2, min_t, max_t, t);
  
  // if either edge (act as ray) hits the triangle, 2 triangles intersect
  bool res = a1_hit || a2_hit || a3_hit || b1_hit || b2_hit || b3_hit;
  return res;
  
  ////////////////////////////////////////////////////////////////////////////
}
