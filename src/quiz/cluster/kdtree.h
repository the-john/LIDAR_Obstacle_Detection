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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)  // double pointer ... node pointer is actually what root was to begin with (Node* root;).  Now we're doing the memory address of this pointer node (double pointer = memory address thats pointing at the node that we are currently adding to the tree) 
	{
		// If the tree is empty, re-assign the node
		if (*node == NULL)  // check if node is NULL.  We de-reference node to see what its value is (i.e. *node).
			*node = new Node(point, id);  // if NULL, de-reference our node and set it (this is now the data that root pointer should be pointing at now)
		else // if not at a NULL node, traverse the tree
		{
			// Calculate the current dim
			uint cd = depth % 2;  // how we traverse the tree depends on the current depth of the tree (an unsigned integer because there are no negative depths).  For a 2D case, depth % 2 will result in and even or odd ... so we will know if the depth is even or odd.  cd will either be zero or one.

			if (point[cd] < ((*node) -> point[cd]))  // if the depth is even (cd is zero so look at the x value)
				insertHelper(&((*node) -> left), depth + 1, point, id);  // branch off to the left child, increment our depth by one, then give it our point and id
			else  // when the depth is odd (cd is one so look at the y value above)
				insertHelper(&((*node) -> right), depth + 1, point, id);  // branch off to the right child, increment our depth by one, then give it our point and id
			
		}
		
	}

	// The insert function modifies the KD-Tree.  This is a recurssive function
	void insert(std::vector<float> point, int id)  // first element in the vector is the x component, second element in the vector is the y component.  "id" is a unique identifier for the point, it's the index that references your point in the point cloud.  We have 11 points in our 2D data, so the id will go from zero to ten.
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root, 0, point, id);  // call insertHelper. resign the node in the tree if it is NULL, so pass in the memory address for root (&root).  Depth is zero (we are at the root).  point and id are needed to create our brand new node.
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




