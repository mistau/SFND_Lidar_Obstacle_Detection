//
// MS KdTree
//
// Michael Staudenmaier, 31Mar20

#pragma once


namespace ms {

  template<typename PointT>
    struct Node
    {
      PointT point;
      int id;
      Node* left;
      Node* right;

    Node(PointT newPoint, int setId)
    :	point(newPoint), id(setId), left(NULL), right(NULL)
      {}
    };

  template<typename PointT>
    struct KdTree
    {
      Node<PointT>* root;

    KdTree()
    : root(NULL)
      {}

      void insert_rec(Node<PointT>** node, uint depth, PointT point, int id)
      {
        if(*node == NULL){
          *node = new Node<PointT>(point, id);
        }else{

          bool cond;
          switch(depth%3){
          case 0:
            cond = point.x < (*node)->point.x;
            break;
          case 1:
            cond = point.y < (*node)->point.y;
            break;
          case 2:
            cond = point.z < (*node)->point.z;
          }
        
          if(cond) {
            insert_rec(&((*node)->left), depth+1, point, id);
          }else{
            insert_rec(&((*node)->right), depth+1, point, id);
          }
        }
      }

      void insert(PointT point, int id)
      {
        insert_rec(&root, 0, point, id);
      }

      void search_rec(PointT target, Node<PointT> *node, uint depth, float distanceTol, std::vector<int>& ids)
      {
        if(node!=NULL){

          // check bounding box
          if( (node->point.x>=(target.x-distanceTol) && node->point.x<=(target.x+distanceTol)) &&
              (node->point.y>=(target.y-distanceTol) && node->point.y<=(target.y+distanceTol)) &&
              (node->point.z>=(target.z-distanceTol) && node->point.z<=(target.z+distanceTol)) )
            {
              float dist = sqrt( (node->point.x - target.x) * (node->point.x - target.x) +
                                 (node->point.y - target.y) * (node->point.y - target.y) +
                                 (node->point.z - target.z) * (node->point.z - target.z) );
              if(dist < distanceTol)
                ids.push_back(node->id);
            }

          // look at additional points
          float valA, valB;
          switch(depth%3){
          case 0:
            valA = target.x;
            valB = node->point.x;
            break;
          case 1:
            valA = target.y;
            valB = node->point.y;
            break;
          case 2:
            valA = target.z;
            valB = node->point.z;
          }
        
          if((valA-distanceTol)<valB)
            search_rec(target, node->left,depth+1, distanceTol, ids);
          if((valA+distanceTol)>valB)
            search_rec(target, node->right,depth+1, distanceTol, ids);
        }
      }
  
      // return a list of point ids in the tree that are within distance of target
      std::vector<int> search(PointT target, float distanceTol)
      {
        std::vector<int> ids;
        search_rec(target, root, 0, distanceTol, ids);
        return ids;
      }
    };

};
