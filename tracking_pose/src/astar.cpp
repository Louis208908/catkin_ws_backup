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


using namespace std;

class MAP{
    ros::NodeHandle nh;
    ros::Subscriber map_server;
    int tempX;
    int tempY;
    double distanceBetweenNode ;
    // the x,y position of a node = -5 + arrayIndex * distanceBetweenNode
    FILE * file;
    FILE * file2;

    public:
        friend class Node;
        double goalX, goalY;
        int mapBuilt;
        int  inflatedMap[100][100];
        pair<double, double> positionMap[100][100];
        list< pair<int,int> > direction;
        list <pair<int,int> > visited;
        MAP(){
            distanceBetweenNode = 0.1;
            map_server = nh.subscribe("map", 1, &MAP::setMap,this);
            file = fopen("/home/louis/Desktop/obstacleMap.csv","w+");
            file2 = fopen("/home/louis/Desktop/map.csv","w+");
            if(file == NULL)
                printf("QAQ");
            mapBuilt = 0;
            for(int dx = 1; dx >= -1; dx --)
                for(int dy = 1; dy >= -1; dy --){
                    if(dx == 0 && dy == 0)
                        continue;
                    direction.push_back(make_pair(dx,dy));     
                }
        }

        void setGoal(double x, double y){
            goalX = x;
            goalY = y;
        }

        bool isGoal(double nowX, double nowY){
            if(fabs(goalX - nowX) < 0.1 && fabs(goalY - nowY) < 0.1)
                return true;
            else
                return false;
        }

        void setMap(const nav_msgs::OccupancyGrid::ConstPtr& map){
            if(!mapBuilt){
                for(int y = 0; y < 100; y ++)
                    for(int x = 0; x < 100; x++){
                        double x_pose =  -5 + ((double)x) * distanceBetweenNode;
                        double y_pose =  -5 + ((double)y) * distanceBetweenNode;
                        positionMap[x][y] = make_pair(x_pose , y_pose); 
                        inflatedMap[x][y] = map->data[100 * y + x];
                    }
                for(int y = 0; y < 100; y ++)
                    for(int x = 0; x < 100; x++)
                        if(map->data[100 * y + x] == 100 || map->data[100 * y + x] == -1)
                            for(int dx = 1; dx >= -1; dx --)
                                for(int dy = 1; dy >= -1; dy --){
                                    tempX = x - dx;
                                    tempY = y - dy;
                                    if((tempX >= 0 && tempX <= 99) && (tempY >= 0 && tempY <= 99))
                                        inflatedMap[tempX][tempY] = 100;
                                }
                for(int y = 99; y >= 0; y--){
                    for(int x = 0; x <= 99; x++){
                        fprintf(file,"%d,",inflatedMap[x][y]);
                        fprintf(file2,"%lf %lf,",positionMap[x][y].first,positionMap[x][y].second);
                    }
                    fprintf(file,"\n");

                    fprintf(file2,"\n");
                }
                mapBuilt = 1;
                cout << "Map built!!" << endl;
             }
         }
        void printList(const list< pair<auto,auto> > &showList){
            for (auto const& i: showList) 
                cout << i.first<< "  " << i.second << "\n";
        }

        pair< double, double>  watchMap(int x, int y){
            return positionMap[x][y];
        }
        double manhattanHeuristic(double nowX, double nowY,double goalX, double goalY){
            double dx,dy;
            dx = goalX - nowX;
            dy = goalY - nowY;
            return fabs(dx) + fabs(dy);
        }

};

class Node{
    double costEstimatedToGoal;
    double costToThisNode;
    double heuristicCost;
    int mapIndexForX, mapIndexForY;
    bool findIndexForMap;
    // in a* costEstimatedToGoal = heuristicCost + costToThisNode

    public:       
        vector <int> actionToThisNode;
        friend class MAP;
        double x,y,theta;
        Node(){
        
        }
        Node(double nodeX, double nodeY, double nodeTheta,double costAccumulated,double heuristic, vector<int> action, MAP map){
            costEstimatedToGoal = costAccumulated + heuristic;
            costToThisNode = 0;
            heuristicCost = 0;
            x = nodeX;
            y = nodeY;
            findIndexForMap = false;
            theta = nodeTheta;
            actionToThisNode = action;
            for(int y = 0; y < 100; y ++){
                for(int x = 0; x < 100; x ++){
                    if( fabs(map.positionMap[x][y].first -nodeX) < 0.1 && fabs(map.positionMap[x][y].second - nodeY) < 0.1){
                        mapIndexForX = x;
                        mapIndexForY = y;
                        findIndexForMap = true;
                        cout << "x,y = " << x << "," << y << endl;
                        break;
                    }
                }
                if(findIndexForMap)
                    break;
            }
        }

        static bool costComparator(const Node nodeA, const Node nodeB){
            return nodeA.costEstimatedToGoal > nodeB.costEstimatedToGoal;
        }
        
        double getEstimatedCost(){
            return costEstimatedToGoal;
        }

        double getAccumulatedCost(){
            return costToThisNode;
        }

        vector<int> getAction(){
            return actionToThisNode;
        }

        
};


double manhattanHeuristic(double nowX, double nowY,double goalX, double goalY){
    double dx,dy;
    dx = goalX - nowX;
    dy = goalY - nowY;
    return fabs(dx) + fabs(dy);
}

list< tuple<double,double,int, double,double> > getSuccessor( Node nowNode, MAP map){
    // the tuple means x ,y ,from which direction , pathCost, heuristic
    int nextX;
    int nextY;
    int which_direction = 1;
    list < tuple<double, double, int ,double,double > > result;
    for(auto it = map.direction.begin(); it != map.direction.end(); it++ ){
        nextX = nowNode.x + (*it).first;
        nextY = nowNode.y + (*it).second;
        if((nextX < 100 && nextX >= 0) && (nextY < 100 && nextY >= 0 )){
            if(map.inflatedMap[nextX][nextY] != 100){
                result.push_back( make_tuple( nextX,nextY,which_direction, nowNode.getAccumulatedCost() + pow( (pow(nextX,2)+pow(nextY,2)),0.5 ), manhattanHeuristic(nowNode.x, nowNode.y,map.goalX,map.goalY) ) );
            }
        }
        which_direction ++;
    }
    return result;
}





int main(int argc, char ** argv){
    ros::init(argc,argv,"pose_tracking");
    MAP newEnviroment;
    while(!newEnviroment.mapBuilt){
        ros::spinOnce(); 
    }
    Node firstNode(0,0,0,0,0,vector<int>(),newEnviroment);
    priority_queue< Node , vector<Node> , decltype(&Node::costComparator) > pq(Node::costComparator);
    pq.push(firstNode);
    Node nowNode = pq.top();
    newEnviroment.visited.push_back(make_pair(firstNode.x,firstNode.y));
    while(!newEnviroment.isGoal(nowNode.x,nowNode.y)){
        Node nowNode = pq.top();
        pq.pop();
        for(auto it :  getSuccessor(nowNode, newEnviroment)){
            nowNode.actionToThisNode.push_back(get<2>(it));
            Node*  newNodePtr = new Node(get<0>(it), get<1>(it),get<2>(it),get<3>(it),get<4>(it),nowNode.actionToThisNode,newEnviroment);
            cout << get<0>(it) << " " << get<1>(it)  << " " << get<2>(it) << endl;
            pq.push(*newNodePtr);
            if(find(make_pair(get<0>(it), get<1>(it)),newEnviroment.visited ) == newEnviroment.visited.end()){
                newEnviroment.visited.push_back(make_pair(get<0>(it),get<1>(it)));
            }
        }
    }
    ros::spin();
    return 0;
}
