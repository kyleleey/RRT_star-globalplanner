#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>

namespace global_planner {
class RRT{
    public:
        RRT();
        RRT(int x, int y);

        struct Node{
            int ID;
            int x;
            int y;
            int parent;
            std::vector<int> child;
        };

        std::vector<Node> getTree();
        void SetTree(std::vector<Node> tree);
        int getTreeSize();

        void add(Node node);
        Node remove(int ID);
        Node get(int ID);

        double getx(int ID);
        double gety(int ID);
        void setx(int ID, double outx);
        void sety(int ID, double outy);
        
        Node getparent(int ID);
        void setparent(int ID, int parentID);

        void setchild(int ID, int childID);
        std::vector<int> getchild(int ID);
        int getchildSize(int ID);

        int getNearest(double X, double Y);
        void getPath(int endID, std::vector<std::pair<float, float> > &path);
        //std::vector<std::pair<float, float> > getPath(int endID);
        void getNear(RRT::Node node, int r, std::vector<int> &near);
        double getCost(int ID);
        double getdistance(int ID, int x, int y); 
    
    private:
        std::vector<Node> Tree;
        double getdistance(double x, double y, double goalx, double goaly);


};


}//end namespace global_planner

