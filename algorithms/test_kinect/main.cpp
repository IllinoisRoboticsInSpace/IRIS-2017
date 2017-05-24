#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>//pthreads
#include <signal.h>
#include <time.h>
#include <sys/time.h> 
#include <math.h> 

//#include <ros/ros.h>

#include "checkboard_navigation_module.h"
#include "data_structure.hpp"
//#include <geometry_msgs/Pose2D.h>

volatile bool positionStringIsUsed = true;
volatile std::string positionsString;
float goal_x,goal_y,goal_t;

using namespace std;

long int millis()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
}

double fmod2pi(double v)
{
    return fmod(fmod(v,M_PI*2)+M_PI*4,M_PI*2);
}

chesspos get_chessboard_navigation_pos()
{
	chesspos ret;
	ret.x=0;
	ret.y=200;
	ret.t=0;
	ret.millis=millis();
	return ret;
}

//Control C handler
void my_handler(int s) {
    cout << "Caught signal "<< s <<"\n";
    exit(1);
}

volatile bool stop_flag = false;

//Create instance of data
navigation_and_mapping_data D;

//void got_theta_IMU(std_msgs::Float32 theta)
//{
    //D.imu_theta=theta.data;
//}

//ros::Publisher right_track_pub;
//ros::Publisher left_track_pub; 

//void got_target_pos(geometry_msgs::Pose2D target_pos)
//{
    ////Actually process path
    //std_msgs::Float32 t_right;
    //t_right.data = v_right;
    //right_track_pub.publish(t_right);
    //std_msgs::Float32 t_left;
    //t_left.data = v_left;
    //left_track_pub.publish(t_left);
//}

void* path_planning(void* unused);
void* FSM(void * unused);

int main(int argc, char **argv)
{
    D.map=0;
    D.local_map=0;
    D.true_pos_x=0;
    D.true_pos_y=0;
    D.true_theta=0;
    D.imu_theta=0;
    D.track_right=0;
    D.track_left=0;

    //control C handling
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //ROS init
    //ros::init(argc, argv, "IRIS_Navigation_and_planning", ros::init_options::NoSigintHandler);

    pthread_t chessboard_t;
    pthread_t navigation_t;
    pthread_t path_planning_t;
    pthread_t fsm_t;

    

    int chessboard = 0; // pthread_create(&chessboard_t, NULL, init_chessboard_navigation, (void*)&stop_flag);
    int navigation = pthread_create(&navigation_t, NULL, init_kinect_mapping, (void*)&stop_flag);
    int path = 0 ; //pthread_create(&path_planning_t, NULL, path_planning, 0);
    int fsm = 0; //pthread_create(&fsm_t, NULL, FSM, 0);
    if(navigation || chessboard || path || fsm)
        exit(EXIT_FAILURE);

    //ros::NodeHandle n("main_estimator");
    //ros::Subscriber sub = n.subscribe("/IRIS/theta_IMU", 1, got_theta_IMU);
    //ros::Subscriber sub = n.subscribe("/IRIS/target_pose", 1,got_target_pos);
    //ros::Publisher current_pos_pub pub = n.advertise<geometry_msgs::Pose2D>("/IRIS/current_pose", 1);

    //ros::Rate r(50);
    while (1) //ros::ok())
    {
        //do the actual estimator
        //geometry_msgs::Pose2D current_pos;
        //current_pos.x=D.true_pos_x;
        //current_pos.y=D.true_pos_y;
        //current_pos.theta=D.true_theta;
        //current_pos_pub.publish(current_pos);
        //ros::spinOnce();
        //r.sleep();
        sleep(10);
    }

    stop_flag = true;

    //pthread_join(chessboard_t, 0);
    pthread_join(navigation_t, 0);
    //pthread_join(path_planning_t, 0);
    //pthread_join(fsm_t, 0);


}
