#include "find_all_intersecting_pairs_using_AABBTrees.h"
#include "box_box_intersect.h"
// Hint: use a list as a queue
#include <list>
#include <iostream>

void find_all_intersecting_pairs_using_AABBTrees(
  const std::shared_ptr<AABBTree> & rootA,
  const std::shared_ptr<AABBTree> & rootB,
  std::vector<std::pair<std::shared_ptr<Object>,std::shared_ptr<Object> > > & 
    leaf_pairs)
{
  ////////////////////////////////////////////////////////////////////////////
  
  // initialize a list
  std::list<std::pair<std::shared_ptr<AABBTree>,std::shared_ptr<AABBTree>>> q;
  
  // root box intersect
  if(box_box_intersect(rootA->box, rootB->box)){
    q.emplace_back(rootA, rootB);
  }
  
  while(!q.empty()){
    // get first pair in the list
    std::shared_ptr<AABBTree> A_sub = q.front().first;
    std::shared_ptr<AABBTree> B_sub = q.front().second;
    q.pop_front();
    
    // when both are leaves subtrees
    if(A_sub->num_leaves <= 2 && B_sub->num_leaves <= 2){
//      leaf_pairs.emplace_back(A_sub, B_sub);
      
      // adds pairs if leaf exists and intersects
      if(A_sub->left && B_sub->left){
        if(box_box_intersect(A_sub->left->box, B_sub->left->box)){
          leaf_pairs.emplace_back(A_sub->left, B_sub->left);
        }
      }
      if(A_sub->left && B_sub->right){
        if(box_box_intersect(A_sub->left->box, B_sub->right->box)){
          leaf_pairs.emplace_back(A_sub->left, B_sub->right);
      
        }
      }
      if(A_sub->right && B_sub->left){
        if(box_box_intersect(A_sub->right->box, B_sub->left->box)){
          leaf_pairs.emplace_back(A_sub->right, B_sub->left);
        }
      }
      if(A_sub->right && B_sub->right){
        if(box_box_intersect(A_sub->right->box, B_sub->right->box)){
          leaf_pairs.emplace_back(A_sub->right, B_sub->right);
        }
      }
    }
    
    // A is leaf node
    else if(A_sub->num_leaves <= 2){
      if(box_box_intersect(A_sub->box, B_sub->left->box)){
        q.emplace_back(A_sub, std::dynamic_pointer_cast<AABBTree>(B_sub->left));
      }
      if(box_box_intersect(A_sub->box, B_sub->right->box)){
        q.emplace_back(A_sub, std::dynamic_pointer_cast<AABBTree>(B_sub->right));
      }
    }
    
    // B is leaf node
    else if(B_sub->num_leaves <= 2){
      if(box_box_intersect(A_sub->left->box, B_sub->box)){
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(A_sub->left), B_sub);
      }
      if(box_box_intersect(A_sub->right->box, B_sub->box)){
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(A_sub->right), B_sub);
      }
    }
    
    // not leaf node
    else{
      if(box_box_intersect(A_sub->left->box, B_sub->left->box)){
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(A_sub->left), std::dynamic_pointer_cast<AABBTree>(B_sub->left));
      }
      if(box_box_intersect(A_sub->left->box, B_sub->right->box)){
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(A_sub->left), std::dynamic_pointer_cast<AABBTree>(B_sub->right));
      }
      if(box_box_intersect(A_sub->right->box, B_sub->right->box)){
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(A_sub->right), std::dynamic_pointer_cast<AABBTree>(B_sub->right));
      }
      if(box_box_intersect(A_sub->right->box, B_sub->left->box)){
        q.emplace_back(std::dynamic_pointer_cast<AABBTree>(A_sub->right), std::dynamic_pointer_cast<AABBTree>(B_sub->left));
      }
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////
}
