#include "point_AABBTree_squared_distance.h"
#include <limits>// std::numeric_limits<double>::infinity();
#include <queue> // std::priority_queue


bool point_AABBTree_squared_distance(
    const Eigen::RowVector3d & query,
    const std::shared_ptr<AABBTree> & root,
    const double min_sqrd,
    const double max_sqrd,
    double & sqrd,
    std::shared_ptr<Object> & descendant)
{
  ////////////////////////////////////////////////////////////////////////////

  // reference: https://en.cppreference.com/w/cpp/container/priority_queue
  // for usage of comparator and priority queue
  
  // initialize priority queue
  auto cmp = [](std::pair<double, std::shared_ptr<AABBTree>> left, std::pair<double, std::shared_ptr<AABBTree>> right){
    return left.first > right.first;
  };
  
  std::priority_queue<std::pair<double, std::shared_ptr<AABBTree>>, std::vector<std::pair<double, std::shared_ptr<AABBTree>>>, decltype(cmp)> q(cmp);
  
  // initialize a queue prioritized by minimum distance
  double sqrd_root = point_box_squared_distance(query, root->box);
  q.emplace(sqrd_root, root);
  
  // initialize minimum distance seen so far
  sqrd = std::numeric_limits<double>::infinity();
  while(!q.empty()){
    
    // d_s: distance from query to subtree's bounding box
    double d_s = q.top().first;
    std::shared_ptr<AABBTree> subtree = q.top().second;
    q.pop();
    
    if(d_s < sqrd){
      // subtree is leaf
      if(subtree->num_leaves <= 2){
        double l_sqrd, r_sqrd;
        std::shared_ptr<Object> l_des, r_des;
        bool l = false;
        bool r = false;
        
        if(subtree->left){
          l = subtree->left->point_squared_distance(query, min_sqrd, max_sqrd, l_sqrd, l_des);
        }
        if(subtree->right){
          r = subtree->right->point_squared_distance(query, min_sqrd, max_sqrd, r_sqrd, r_des);
        }
        
        if(l && l_sqrd < sqrd){
          sqrd = l_sqrd;
          descendant = subtree->left;
        }
        if(r && r_sqrd < sqrd){
          sqrd = r_sqrd;
          descendant = subtree->right;
        }
      }
      
      // insert left right subtree to queue
      else{
        double d_l, d_r;
        
        d_l = point_box_squared_distance(query, subtree->left->box);
        d_r = point_box_squared_distance(query, subtree->right->box);
        
        q.emplace(d_l, std::dynamic_pointer_cast<AABBTree>(subtree->left));
        q.emplace(d_r, std::dynamic_pointer_cast<AABBTree>(subtree->right));
      }
    }
  }
  
  return descendant != NULL;
  
  ////////////////////////////////////////////////////////////////////////////
}
