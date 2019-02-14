#include "AABBTree.h"
#include "insert_box_into_box.h"

# include <iostream>

AABBTree::AABBTree(
  const std::vector<std::shared_ptr<Object> > & objects,
  int a_depth): 
  depth(std::move(a_depth)), 
  num_leaves(objects.size())
{
  ////////////////////////////////////////////////////////////////////////////

    int n = objects.size();
    
    // create boudingbox to be the overall bouding box
    this->box = objects[0]->box;
    for(int i=0; i<n; i++){
      insert_box_into_box(objects[i]->box, this->box);
    }
    
    // base case: n = 1
    if(n == 1){
      this->left = objects[0];
      this->right = NULL;
//      insert_box_into_box(this->left->box, this->box);
    }
    // base case: n = 2
    else if(n == 2){
      this->left = objects[0];
      this->right = objects[1];
//      insert_box_into_box(this->left->box, this->box);
//      insert_box_into_box(this->right->box, this->box);
    }
    else{
      
      // find the longest axis
      Eigen::RowVector3d axis_lens = this->box.max_corner - this->box.min_corner;
      int longest_idx;
      int longest_axis = axis_lens.maxCoeff(&longest_idx);
      double midpoint = 0.5 * (this->box.max_corner[longest_idx] + this->box.min_corner[longest_idx]);
      
      // initialize sub trees
      std::vector<std::shared_ptr<Object> > left_sub, right_sub;
      for(int i=0; i<n; i++){
        // partition current object into left
        if(objects[i]->box.center()[longest_idx] < midpoint){
          left_sub.push_back(objects[i]);
        }
        else{
          right_sub.push_back(objects[i]);
        }
      }
      
      // zero partition
      if(left_sub.size() == 0){
        left_sub.push_back(right_sub.back());
        right_sub.pop_back();
      }
      else if(right_sub.size() == 0){
        right_sub.push_back(left_sub.back());
        left_sub.pop_back();
      }
      
      // recursively generate subtree
      this->left = std::make_shared<AABBTree>(left_sub, a_depth + 1);
      this->right = std::make_shared<AABBTree>(right_sub, a_depth + 1);
      
      // extend box
//      insert_box_into_box(this->left->box, this->box);
//      insert_box_into_box(this->right->box, this->box);
      
    }
        
  ////////////////////////////////////////////////////////////////////////////
}

bool AABBTree::ray_intersect(
  const Ray& ray,
  const double min_t,
  const double max_t,
  double & t,
  std::shared_ptr<Object> & descendant) const 
{
  ////////////////////////////////////////////////////////////////////////////

  // hit the box
  if(ray_intersect_box(ray, this->box, min_t, max_t)){
    
    // hit for left and right
    double l_t, r_t;
    std::shared_ptr<Object> l_des, r_des;
    bool l_hit = (this->left != NULL) && (this->left->ray_intersect(ray, min_t, max_t, l_t, l_des));
    bool r_hit = (this->right != NULL) && (this->right->ray_intersect(ray, min_t, max_t, r_t, r_des));
      
    // hit leaf node
    if(l_hit && this->num_leaves <= 2){
      l_des = this->left;
    }
    if(r_hit && this->num_leaves <= 2){
      r_des = this->right;
    }
    
    if(l_hit && r_hit){
      if(l_t < r_t){
        t = l_t;
        descendant = l_des;
      }
      else{
        t = r_t;
        descendant = r_des;
      }
      return true;
    }
    else if(l_hit){
      t = l_t;
      descendant = l_des;
      return true;
    }
    else if(r_hit){
      t = r_t;
      descendant = r_des;
      return true;
    }
    else{
      return false;
    }
  }

  return false;
  
  ////////////////////////////////////////////////////////////////////////////
}
