/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>
#include <vector>

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
		
        Node *tmp_root{root}, *prev{nullptr};
        
        int depth{0};
        bool right{false};
        Node *n = new Node{point, id};
        
        if (root == NULL){
            root = n;
            return;
        }
        
        while (tmp_root){
            prev = tmp_root;
            if(n->point.at(depth % 2) >= tmp_root->point.at(depth % 2)){
                tmp_root = tmp_root->right;
                right = true;
            }else{
                tmp_root = tmp_root->left;
                right = false;
            }
            depth++;
        }
        
        if(right){
            prev->right = n;
        }else{
            prev->left = n;
        }
   	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
        std::vector<int> ids;
        int depth{0};
        Node *tmp_root{nullptr};
        std::vector<Node*> stack;
        stack.push_back(root);
        
        while(!stack.empty()){
            tmp_root = std::move(stack.back());
            stack.pop_back();
            if (in_box(tmp_root, target, distanceTol) && distance(tmp_root, target, distanceTol)){
                ids.push_back(tmp_root->id);
                
            }

            if (target.at(depth % 2) - distanceTol < tmp_root->point.at(depth % 2)){
                if(tmp_root->left)
                    stack.push_back(tmp_root->left);
            }
            if(target.at(depth % 2) + distanceTol >= tmp_root->point.at(depth % 2)){
                if(tmp_root->right)
                    stack.push_back(tmp_root->right);
            }
            depth++;
        }
        
        return ids;
	}
    
    bool in_box(Node *n, std::vector<float> &target, float &distanceTol){
        float x_l{target.at(0) - distanceTol}, x_r{target.at(0) + distanceTol}, y_up{target.at(1) + distanceTol}, y_down{target.at(1) - distanceTol};
        if ((n->point.at(0) <= x_r && n->point.at(0) >= x_l) && (n->point.at(1) <= y_up && n->point.at(1) >= y_down))
            return true;
        return false;
    }
	
    bool distance(Node *other, std::vector<float> &target, float &distanceTol){
        return sqrt(pow(target.at(0) - other->point.at(0), 2) + pow(target.at(1) - other->point.at(1), 2)) <= distanceTol;
    }

};




