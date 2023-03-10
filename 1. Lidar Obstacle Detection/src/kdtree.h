
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_

// Structure to represent node of kd tree
struct Node
{
  std::vector<float> point;
  int id;
  Node* left=NULL;
  Node* right=NULL;

  Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL)
  {
  }
};

struct KdTree
{
  Node* root=NULL;

  KdTree() : root(NULL)
  {
  }

  void insertHelper(Node** node, int depth, std::vector<float> point, int id)
  {
    if(*node == nullptr)
    {
      *node = new Node(point, id);
    }
    else
    {
      int dim = depth % 3;
      if(point[dim] < (*node)->point[dim])
      {
        insertHelper(&(*node)->left, depth + 1, point, id);
      }
      else
      {
          insertHelper(&(*node)->right, depth + 1, point, id);
      }
    }
  }

  void insert(std::vector<float> point, int id)
  {
    insertHelper(&root, 0, point, id);
  }

  std::vector<int> searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol)
  {
      std::vector<int> IDs;
      float leftBound = target[0] - distanceTol;
      float rightBound = target[0] + distanceTol;
      float topBound = target[1] + distanceTol;
      float bottomBound = target[1] - distanceTol;
      float backBound = target[2] - distanceTol;
      float frontBound = target[2] + distanceTol;

      if(node != nullptr)
      {
        if((node->point[0] >= leftBound   && node->point[0] <= rightBound) && 
           (node->point[1] >= bottomBound && node->point[1] <= topBound)   && 
           (node->point[2] >= backBound   && node->point[2] <= frontBound))
        {
          float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2] - target[2], 2));
          if(distance <= distanceTol)
          {
            IDs.push_back(node->id);
          }
        }

        if(target[depth%3] - distanceTol < node->point[depth%3])
        {
          std::vector<int> leftIDs = searchHelper(target, node->left, depth + 1, distanceTol);
          IDs.insert(IDs.end(), leftIDs.begin(), leftIDs.end());
        }
        if(target[depth%3] + distanceTol > node->point[depth%3])
        {
          std::vector<int> rightIDs = searchHelper(target, node->right, depth + 1, distanceTol);
          IDs.insert(IDs.end(), rightIDs.begin(), rightIDs.end());
        }
      }

      return IDs;
  }

  std::vector<int> search(std::vector<float> target, float distanceTol)
  {
    std::vector<int> IDs;
		IDs = searchHelper(target, root, 0, distanceTol);
		return IDs;
  }
};

#endif
