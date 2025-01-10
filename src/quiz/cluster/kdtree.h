/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		Node** node = &root;
		int depth = 0;

		while (*node != NULL) {
			// Determine current dimension (assuming 2D points)
			int tree_dim = depth % 2;

			// Traverse left or right based on the current dimension
			if (point[tree_dim] < ((*node)->point[tree_dim])) {
				node = &((*node)->left);
			} else {
				node = &((*node)->right);
			}

			depth++;
		}

		// Insert the new node
		*node = new Node(point, id);
	
	}

	
	void searchHelper(Node* node, std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids) {
		if (node == NULL) return;

		// Check if the node is within the target bounding box
		if (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) &&
			node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) {

			float distance = sqrt(pow((node->point[0] - target[0]), 2) + pow((node->point[1] - target[1]), 2));
			if (distance <= distanceTol) {
				ids.push_back(node->id);
			}
		}

		// Determine current dimension (assuming 2D points)
		int tree_dim = depth % 2;

		// Recursively search children based on their bounds
		if (target[tree_dim] - distanceTol < node->point[tree_dim]) {
			searchHelper(node->left, target, distanceTol, depth + 1, ids);
		}
		if (target[tree_dim] + distanceTol > node->point[tree_dim]) {
			searchHelper(node->right, target, distanceTol, depth + 1, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
	

};




