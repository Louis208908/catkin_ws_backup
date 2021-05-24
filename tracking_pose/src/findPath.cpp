#include "bits/stdc++.h"
#include "math.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "stdio.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"
#include <nav_msgs/OccupancyGrid.h>
#include "time.h"
#include "tracking_pose/ask_path.h"


using namespace std;

#define endl "\n"



inline bool isGoal(double nowX, double nowY, double goalX, double goalY){
    if(fabs(goalX - nowX) < 0.05 && fabs(goalY - nowY) < 0.05)
        return true;
    else
        return false;
}

inline double ChebyshevHeuristic(double pose_x, double pose_y, double goalX, double goalY){
    double dx,dy;
    dx = goalX - pose_x;
    dy = goalY - pose_y;
    
    return  0.1414 * (min(dx,dy)) + max(dx,dy) - min(dx,dy);
    //return 0;
    //return pow(dx,2) + pow(dy,2);
}


struct MapInformation{
    double x,y;
    bool obstacleExist;
};


class Node{
    ros::Subscriber map_server;
    ros::ServiceServer aStarServer;
    FILE  *file, *file2;

    public:
        int xIndexAtMap, yIndexAtMap;
        ros::NodeHandle nh;
        bool findIndexForMap;
        double x_init,y_init,theta;
        double costEstimatedToGoal;
        double costToThisNode;
        double heuristicCost;
        vector <int> actionToThisNode;
        vector <pair<int,int> > path;


        // static members
        static bool mapBuilt;
        static int  expandCount;
        static list< pair<int,int> > direction;
        constexpr static double distanceBetweenNode = 0.1;
        static int nodeVisitedAmount;
        static struct MapInformation mapInformation[100][100];


        Node(double heuristic){
            costEstimatedToGoal =    heuristic;
        }
        Node(){
            map_server = nh.subscribe("map", 1, &Node::setMap,this);
            aStarServer = nh.advertiseService("ask_path",&Node::answerPath,this);
            for(int dx = 1; dx >= -1; dx --)
               for(int dy = 1; dy >= -1; dy --){
                   if(dx == 0 && dy == 0)
                       continue;
                   Node::direction.push_back(make_pair(dx,dy));     
               }
        }
        Node(double x, double y){
            
            costToThisNode = 0;
            costEstimatedToGoal = 0;
            findIndexForMap = false;
            actionToThisNode.reserve(10000) ;
            path.reserve(10000);   
            findIndex(x,y);
            //path.push_back(make_pair(xIndexAtMap,yIndexAtMap));
            // cout << xIndexAtMap << "," << yIndexAtMap << endl;
            
        }
        // initializing first Node

        void findIndex(double nowX, double nowY){
            for(int y = 0; y < 100; y ++){
                for(int x = 0; x < 100; x ++){
                    if( fabs(Node::mapInformation[x][y].x -nowX) < 0.05 && fabs(Node::mapInformation[x][y].y - nowY) < 0.05){
                        xIndexAtMap = x;
                        yIndexAtMap = y;
                        findIndexForMap = true;
                        break;
                    }
                }
                if(findIndexForMap)
                    break;
            }
        }

        Node(double x, double y, vector<int> directionToHere, double pathCost, double heuristic,vector<pair<int,int> > fromWhere){
            xIndexAtMap = x;
            yIndexAtMap = y;
            actionToThisNode = directionToHere;
            costToThisNode = pathCost;
            costEstimatedToGoal = costToThisNode + heuristic;
            path = fromWhere;
            
        }
        //initializing new node

        void setMap(const nav_msgs::OccupancyGrid::ConstPtr& map){
            if(!Node::mapBuilt){
                file = fopen("/home/louis/Desktop/map.csv","w+");
                file2 = fopen("/home/louis/Desktop/obstacle.csv","w+");
                for(int y = 0; y < 100; y ++)
                    for(int x = 0; x < 100; x++){
                        double x_position =  -5 + x * Node::distanceBetweenNode;
                        double y_position =  -5 + y * Node::distanceBetweenNode;
                        Node::mapInformation[x][y].x = x_position;
                        Node::mapInformation[x][y].y = y_position;
                        Node::mapInformation[x][y].obstacleExist = map->data[100 * y + x];
                    }
                for(int y = 0; y < 100; y ++)
                    for(int x = 0; x < 100; x++)
                        if(map->data[100 * y + x] == 100 || map->data[100 * y + x] == -1)
                            for(int dx = 2; dx >= -2; dx --)
                                for(int dy = 2; dy >= -2; dy --){
                                    int tempX = x - dx;
                                    int tempY = y - dy;
                                    if((tempX >= 0 && tempX <= 99) && (tempY >= 0 && tempY <= 99))
                                        Node::mapInformation[tempX][tempY].obstacleExist = 100;
                                }
                for(int y = 99; y >= 0; y--){
                    for(int x = 0; x <= 99; x++){
                        fprintf(file,"%d,",Node::mapInformation[x][y].obstacleExist);
                        fprintf(file2,"%lf %lf,",Node::mapInformation[x][y].x,Node::mapInformation[x][y].y);                        
                    }
                    fprintf(file,"\n");
                    fprintf(file2,"\n");
                }
                Node::mapBuilt = 1;
             }
        }

        static bool costComparator(const Node nodeA, const Node nodeB){
            return nodeA.costEstimatedToGoal > nodeB.costEstimatedToGoal;
        }

        // void setGoal(double x, double y){
        //     Node::goalX = x;
        //     Node::goalY = y;
        // }

        static vector<Node>  getSuccessor(Node nowNode,double goal_x,double goal_y){
            int nextX;
            int nextY;
            int which_direction = 1;
            double cost;
            vector<Node> result;
            result.reserve(8);
            double heuristicCost;
            for(auto it = Node::direction.begin(); it != Node::direction.end(); it++ ){
                nextX = nowNode.xIndexAtMap + it->first;
                nextY = nowNode.yIndexAtMap + it->second;
                if((nextX < 100 && nextX >= 0) && (nextY < 100 && nextY >= 0 )){
                    if(!Node::mapInformation[nextX][nextY].obstacleExist ){
                        Node::expandCount ++;
                        cost = pow( pow(it->first / 10.0,2) + pow(it->second / 10.0 ,2) ,0.5);
                        heuristicCost = ChebyshevHeuristic(Node::mapInformation[nextX][nextY].x,
                                                            Node::mapInformation[nextX][nextY].y
                                                            ,goal_x
                                                            ,goal_y);
                        vector<int> act = nowNode.actionToThisNode;
                        if(!act.empty()){
                            if(*(act.end() - 1) != which_direction)
                                //heuristicCost += (45 / 360) * 2 * M_PI / 10;
                                heuristicCost *= 1.005;
                        }
                        vector< pair<int, int> > fromWhere = nowNode.path;
                        fromWhere.push_back(make_pair(nextX,nextY));
                        //this will tell us the index for mapMatrix
                        act.push_back(which_direction);
                        Node* nodePtr =  new Node(nextX,nextY,act,nowNode.costToThisNode + cost,heuristicCost, fromWhere);
                        result.push_back(*nodePtr);
                    }
                }
                which_direction ++;
            }
            return result;
        }
        void showMap(double goal_x, double goal_y){
            for(int y = 99; y >= 0; y--)
                for(int x = 0; x <= 99; x++){
                    pair<int,int> printWhere =  make_pair(x,y);
                    auto temp = find(path.begin(),path.end(),printWhere);
                    if(temp == path.end()){
                        if(Node::mapInformation[x][y].obstacleExist)
                            printf("x");
                        else 
                            printf(".");
                    }
                    else{
                        if(!isGoal(Node::mapInformation[x][y].x,Node::mapInformation[x][y].y,goal_x,goal_y))
                            printf("*");
                        else
                            printf("!");
                    }
                }
            printf("\n");
        }

        static Node findPath(double initX, double initY,double goalX, double goalY){
            priority_queue< Node , vector<Node> , decltype(&Node::costComparator) > pq(Node::costComparator);
            bool visited[100][100];
            memset(visited,0,100 * 100);
            Node startNode(initX,initY);
            pq.push(startNode);
            clock_t start = clock();
            while(!pq.empty()){
                Node nowNode = pq.top();
                pq.pop();
                if(!isGoal(Node::mapInformation[nowNode.xIndexAtMap][nowNode.yIndexAtMap].x,Node::mapInformation[nowNode.xIndexAtMap][nowNode.yIndexAtMap].y,goalX,goalY)){
                    if(!visited[nowNode.xIndexAtMap][nowNode.yIndexAtMap]){
                        Node::nodeVisitedAmount ++;
                        visited[nowNode.xIndexAtMap][nowNode.yIndexAtMap] = true;
                        auto successors = getSuccessor(nowNode,goalX,goalY);
                        for(auto it : successors)
                            if(!visited[it.xIndexAtMap][it.yIndexAtMap])
                                pq.push(it);
                    }
                }
                else{
                    nowNode.path.push_back(make_pair(nowNode.xIndexAtMap,nowNode.yIndexAtMap));
                    clock_t end = clock();
                    cout << "time cost = " << (double)(end-start)/CLOCKS_PER_SEC << endl;
                    cout << "expand count = " << Node::expandCount << endl;
                    cout << "node visited = "  << Node::nodeVisitedAmount << endl;  
                    //FILE *file3 = fopen("/home/louis/Desktop/visited.csv","w+");
                    //for(auto it = Node::visited.begin(); it != Node::visited.end(); it++){
                    //    fprintf(file3, "%lf %lf\n",it->first,it->second);
                    //}
                    for(auto it = nowNode.actionToThisNode.begin(); it != nowNode.actionToThisNode.end(); it++){
                        cout << *it ;
                        if(it != nowNode.actionToThisNode.end() - 1)
                            cout << "->";
                    }
                    cout << endl;
                    int qq = 0;
                    for(auto it = nowNode.path.begin(); it != nowNode.path.end(); it++){
                        cout << "("<< it->first <<","<<it->second << ")";
                        if(it != nowNode.path.end() - 1)
                            cout << "->";
                    }
                    cout << endl;
                    nowNode.showMap(goalX,goalY);
                    int count = 0;
                
                    return nowNode;
                }
            }
        }
        vector< pair<double, double> >indexToMap(vector< pair<int, int> > index){
            vector< pair<double, double> > toMap;
            toMap.reserve(1000);
            for(auto it = index.begin(); it != index.end(); it++){
                toMap.push_back( make_pair(Node::mapInformation[it->first][it->second].x,Node::mapInformation[it->first][it->second].y));
            }
            return toMap;
        }

        vector< pair<int,int> >refineMap(vector< pair<int,int> >  originMap ){
            double slope = 0;
            int previousY = 0;
            int previousX = 0;
            double previousSlope = 0;
            for(auto it = originMap.begin(); it != originMap.end(); ){
                slope = (it->second - previousY) / (it->first - previousX);
                previousY = it->second;
                previousX = it->first;
                if(slope == previousSlope){
                    auto deleteLater = it++;
                    originMap.erase(deleteLater);
                }
                else{
                    it++;
                    previousSlope = slope;
                }
                
            }
            return originMap;
        }

        vector< int > refineDirection(vector<int> originAct){
            int previousAct = 0;
            for(auto it = originAct.begin(); it != originAct.end();){
                if(*it == previousAct){
                    auto deleteLater = it++;
                    originAct.erase(it);
                }
                else{
                    it++;
                    previousAct = *it;
                }
            }
            return originAct;
        }

        bool answerPath(tracking_pose::ask_path::Request &req, tracking_pose::ask_path::Response &res){
            // system("clear");
            Node finalNode  = Node::findPath(req.now_x,req.now_y,req.goal_x,req.goal_y);
            //auto finalPath = refineMap(finalNode.path);
            //auto pathOnMap = indexToMap(finalPath);
            auto pathOnMap = indexToMap(finalNode.path);
            for(auto it = pathOnMap.begin(); it != pathOnMap.end(); it++){
                res.nextPointX.push_back(it->first);
                res.nextPointY.push_back(it->second);
                // it++;
                // cout << "("<< it->first <<","<<it->second << ")";
                // if(it != nowNode.path.end() - 1)
                    // cout << "->";
            }
            //auto finalAct = refineDirection(finalNode.actionToThisNode);
            //for(auto it:finalAct){
            //    res.nextPointDirection.push_back(it);
            //}
            for(auto it:finalNode.actionToThisNode){
                res.nextPointDirection.push_back(it);
            }

            return true; 
        }
};






bool Node::mapBuilt = false;
list< pair<int,int> > Node::direction;
int Node::expandCount = 0;
int Node::nodeVisitedAmount = 0;
struct MapInformation Node::mapInformation[100][100];

int main(int argc, char ** argv){
    ros::init(argc,argv,"find_path");
    
    double goalX,goalY;
    double init_x = stod(argv[1]);
    double init_y = stod(argv[2]);
    // argc means number of argument
    // argv is an array of string which stores arguments
    // argv[0] is the program name
    Node initNode;
    initNode.nh.getParam("goalX",goalX);
    initNode.nh.getParam("goalY",goalY);
    while(!Node::mapBuilt)
        ros::spinOnce();
    Node::findPath(init_x,init_y,goalX,goalY);
    while(ros::ok()){
        ros::spinOnce();
    }
}
