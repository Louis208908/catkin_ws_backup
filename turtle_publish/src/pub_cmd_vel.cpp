#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"
#include "stdio.h"

double degree;
double deg_err;
double targetX;
double targetY;
double targetDeg;
short  turtle_state;
short  arrival;

void loop_pose(const turtlesim::Pose::ConstPtr& pose){
    printf("x = %f\n",pose->x);
    printf("y = %f\n",pose->y);
    degree = pose->theta * 57.3;
    if(degree > 360)
        degree -= 360;
    else if(degree < 0)
        degree += 360;
    //printf("theta = %f\n",pose->theta);
    printf("deg = %lf\n",degree);
}



int main(int argc, char ** argv){
	ros::init(argc,argv,"turtle_publish");
    ros::NodeHandle n;
    ros::Publisher publish = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    ros::Subscriber pose = n.subscribe("turtle1/pose", 1000, loop_pose);
    int rate = 5;
    ros::Rate loop_rate(rate);
    int count = 0;
    geometry_msgs::Twist msg;
    while(ros::ok()){
        if(count <= 10){
            count ++;
            msg.linear.x = 0.2 * rate;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        }
        else if(count > 10 && count <= 20){
            count ++;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = ((3.1415926535897 / 2) / 10) * rate;
        }
        else{
            count = 0;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = 0;
        }
        loop_rate.sleep();
        publish.publish(msg);
    
        ros::spinOnce();
    }              
                   
    return 0;
}                  
                   
