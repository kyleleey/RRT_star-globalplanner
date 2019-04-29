#include<global_planner/rrt.h>
#include<costmap_2d/cost_values.h>
#include<math.h>

namespace global_planner {

RRT::RRT()
{
    RRT::Node newNode;
    newNode.ID = 0;
    newNode.x = 0;
    newNode.y = 0;
    newNode.parent = 0;

    Tree.push_back(newNode);
}

RRT::RRT(int x, int y)
{
    RRT::Node newNode;
    newNode.ID = 0;
    newNode.x = x;
    newNode.y = y;
    newNode.parent = 0;
        
    Tree.push_back(newNode);
}

std::vector<RRT:: Node> RRT::getTree()
{
    return Tree;
}

void RRT::SetTree(std::vector<Node> tree)
{
    Tree = tree;
}

int RRT::getTreeSize()
{
    return Tree.size();
}

void RRT::add(Node node)
{
    Tree.push_back(node);
}

RRT::Node RRT::remove(int ID)
{
    RRT::Node temp = Tree[ID];
    Tree.erase(Tree.begin() + ID);
    return temp;
}

RRT::Node RRT::get(int ID)
{
    return Tree[ID];
}

double RRT::getx(int ID)
{
    return Tree[ID].x;
}

double RRT::gety(int ID)
{
    return Tree[ID].y;
}


void RRT::setx(int ID, double outx)
{
    Tree[ID].x = outx;
}

void RRT::sety(int ID, double outy)
{
    Tree[ID].y = outy;
}

RRT::Node RRT::getparent(int ID)
{
    return Tree[Tree[ID].parent];
}

void RRT::setparent(int ID, int parentID)
{
    Tree[ID].parent = parentID;
}

void RRT::setchild(int ID, int childID)
{
    Tree[ID].child.push_back(childID);
}

std::vector<int> RRT::getchild(int ID)
{
    return Tree[ID].child;
}

int RRT::getchildSize(int ID)
{
    return Tree[ID].child.size();
}

int RRT::getNearest(double X, double Y)
{
    int i,id;
    double distance = 1e10;
    double temp;
    for(i=0; i<this->getTreeSize();i++)
    {
        temp = getdistance(X, Y, getx(i), gety(i));
        if (temp < distance)
        {
            distance = temp;
            id = i;
        }
    }
    return id;
}

double RRT::getdistance(double x, double y, double goalx, double goaly)
{
    return sqrt(pow(goalx - x,2) + pow(goaly - y,2));
}

double RRT::getdistance(int ID, int x, int y)
{
    int deltax = getx(ID) - x;
    int deltay = gety(ID) - y;
    
    return sqrt(pow(deltax,2) + pow(deltay, 2));
}

void RRT::getPath(int endID, std::vector<std::pair<float, float> > &path)
{
    std::pair<float, float> current;
    int currentID = endID;
    current.first = Tree[endID].x;
    current.second = Tree[endID].y;
    path.push_back(current);
    while(Tree[currentID].parent != 0)
    {
        current.first = Tree[Tree[currentID].parent].x;
        current.second = Tree[Tree[currentID].parent].y;
        path.push_back(current);
        currentID = Tree[currentID].parent;
    }
    current.first = Tree[0].x;
    current.second = Tree[0].y;
    path.push_back(current);
}

 void RRT::getNear(RRT::Node node, int r, std::vector<int> &near)
{
    near.clear();
    int distance;
    for(int i=0; i<this->getTreeSize();i++)
    {
        distance = getdistance(getx(i), gety(i), node.x, node.y);
        if( distance < r )
            near.push_back(i);
    }
}

double RRT::getCost(int ID)
{
    double cost = 0;
    while(Tree[ID].parent != 0)
    {
        cost = cost + getdistance(getx(ID), gety(ID), getx(Tree[ID].parent), gety(Tree[ID].parent)); 
        ID = Tree[ID].parent;
    }
    cost = cost + getdistance(getx(ID), gety(ID), getx(Tree[ID].parent), gety(Tree[ID].parent));
    return cost;
}

}//end namespace global_planner
