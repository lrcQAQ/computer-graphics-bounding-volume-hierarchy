#include "ray_intersect_triangle.h"

bool ray_intersect_triangle(
  const Ray & ray,
  const Eigen::RowVector3d & A,
  const Eigen::RowVector3d & B,
  const Eigen::RowVector3d & C,
  const double min_t,
  const double max_t,
  double & t)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // construct linear system
  double a, b, c, d, e, f, g, h, i, j, k, l;
  
  a = A[0] - B[0];
  b = A[1] - B[1];
  c = A[2] - B[2];
  d = A[0] - C[0];
  e = A[1] - C[1];
  f = A[2] - C[2];
  g = ray.direction[0];
  h = ray.direction[1];
  i = ray.direction[2];
  
  j = A[0] - ray.origin[0];
  k = A[1] - ray.origin[1];
  l = A[2] - ray.origin[2];
  
  // compute reuse terms
  double ei_hf, gf_di, dh_eg, ak_jb, jc_al, bl_kc;
  
  ei_hf = e * i - h * f;
  gf_di = g * f - d * i;
  dh_eg = d * h - e * g;
  ak_jb = a * k - j * b;
  jc_al = j * c - a * l;
  bl_kc = b * l - k * c;
  
  // using Cramer's rule to solve linear systems
  double M = a * ei_hf + b * gf_di + c * dh_eg;
  
  // compute t
  t = -(f * ak_jb + e * jc_al + d * bl_kc) / M;
  if(t < min_t || t >= max_t){
    return false;
  }
  
  // compute gamma
  double gamma = (i * ak_jb + h * jc_al + g * bl_kc) / M;
  if(gamma < 0 || gamma > 1){
    return false;
  }
  
  // compute beta
  double beta = (j * ei_hf + k * gf_di + l * dh_eg) / M;
  if(beta < 0 || beta > 1-gamma){
    return false;
  }
  
  return true;
  
  ////////////////////////////////////////////////////////////////////////////
}

