#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "checkboard_navigation_module.h"
#include "data_structure.hpp"
//#include <ros/ros.h>
//#include "std_msgs/String.h"
#include <sstream>

using namespace std;

//#define pow2(x) (x)*(x)

const double LINEAR_CONST = 1000/2.;
const double ANGULAR_CONST = 1000/0.05;

//Global variables
volatile double goal_x;
volatile double goal_y;

enum{RETRACT = 0, STAY = 1, EXTEND = 2, STOP = 0, MOVE = 1, BACKWARDS = -2};

volatile int bin_movement = 1; // 0=RETRACT 1=STAY 2=EXTEND
volatile int paddle_movement = 1; // 0=RETRACT 1=STAY 2=EXTEND
volatile int paddle_onoff = 0; // 0=STOP 1=MOVE


volatile int control_direction=1;

//absolute value templated
template<typename T> T absd(T d){return d<0?-d:d;}
//efficient square templated
template<typename T> T pow2(T d){return d*d;}
//find distance in S(2pi) set (like a circle: through either side)
template<typename T> T diff2pi(T d)
{
    d=fmod2pi(d);
    return min(d,2*M_PI-d);
}

void* path_planning(void* unused)
{
    //Initialize ROS node and publisher
    int count_loops=0;
//    ros::NodeHandle n;
//    ros::Publisher pub_control=n.advertise<std_msgs::String>("/IRIS/autonomous_command", 1);
    chesspos poss = {0,0,0,0};
    while(poss.millis==0) //wait for first location
        poss = get_chessboard_navigation_pos();
    while(1)
    {
        double forward_cntl;
        double turning_cntl;
        //Get robot position
        chesspos pos = get_chessboard_navigation_pos();
        double right;
        double left;
        
        if(millis()-pos.millis<2500)
        {
            //Message setup
            if(control_direction==BACKWARDS)
            {
                forward_cntl=-1000;
                turning_cntl = 0.;
            }
            else
            {
                forward_cntl = control_direction*sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y))*LINEAR_CONST;
                if(control_direction>0)
                    turning_cntl = diff2pi(fmod2pi(atan2(goal_y - pos.y, goal_x - pos.x)) - fmod2pi(pos.t))*ANGULAR_CONST;
                else if(control_direction<0)
                    turning_cntl = diff2pi(fmod2pi(atan2(goal_y - pos.y, goal_x - pos.x)) - fmod2pi(pos.t+M_PI))*ANGULAR_CONST;
                else
                    turning_cntl = 0.;
            }
            
            //normalize and get right and left values
            double normalizer=absd(turning_cntl)+absd(forward_cntl);
            if(normalizer>1000.)
            {
                turning_cntl/=normalizer/1000.;
                forward_cntl/=normalizer/1000.;
            }
            right=forward_cntl-turning_cntl;
            left=forward_cntl+turning_cntl;
        }
        else
        {
            if((count_loops%20)==1)
                std::cout<<"\033[0;32m"<< "PATHPLAN: ********* position is too old ******** "<<"\033[0m\n";
            right=0;
            left=0;
        }
        //publish messages
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << left << "," << right << "," << bin_movement << "," << paddle_movement << "," << paddle_onoff ;
//        msg.data = ss.str();
//        pub_control.publish(msg);
        
        usleep(50000);
        count_loops++;
        if((count_loops%20)==1)
        {
            std::cout<<"\033[0;32m"<< "PATHPLAN: current "<<pos.x<<" "<<pos.y<<" "<<pos.t<<" target "<< goal_x << " " << goal_y << " action l "<< left << " r " << right <<"\033[0m\n";
        }
    }
}

//Sets a goal to move to
void set_goal(double x, double y, int dir, const char * comment="")
{
    std::cout<<"\033[0;35m"<< "PATHPLAN: set_goal "<<x<<" "<<y<<" "<<dir<<" "<< comment <<"\033[0m\n";
    //Set the goal position to the x and y values
    goal_x = x;
    goal_y = y;
    control_direction = dir;
}

//Waits until robot reaches destination within specified tolerance
void wait_for_dist(double epsilon, const char * comment="")
{
    std::cout<<"\033[0;35m"<< "PATHPLAN: wait_for_dist "<< epsilon << " " << comment <<"\033[0m\n";
    //Wait in here until the robot position is at or within the allowed tolerance
    chesspos pos = get_chessboard_navigation_pos();
    double dist = sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y));

    while(dist > epsilon)
    {
        //Recompute distance from goal
        pos = get_chessboard_navigation_pos();
        dist = sqrt(pow2(goal_x - pos.x) + pow2(goal_y - pos.y));

        //Wait for 10 ms
        usleep(10000);
    }

    control_direction = 0;
}

//Main Finite State Machine
void* FSM(void * unused)
{
    //Sequentially move through the different states: move_to_mine -> mine -> move_to_deposit -> deposit
    //Offset for varying the x-axis position of the goal states and iteration level
    int offset[3] = {0, 100, -100};
    int iter = 0;

    double x;
    double y;
    double epsilon = 100;

    while(1)
    {
        //Move to mine
        x = offset[iter];
        y = 500;
        //epsilon = 0.2*y;
        set_goal(x, y, 1, "Move to mine");
        wait_for_dist(epsilon,  "Move to mine");

        //Mine
        //Order: start Maxon -> lower paddle -> set_goal() -> wait_for_dist() -> raise paddle -> stop Maxon
        x = offset[iter];
        y += 50;
        //epsilon = 0.2*y;
        
        paddle_onoff = MOVE;
        paddle_movement = RETRACT;
        set_goal(x, y, 1, "Mine");
        sleep(10); //???
        paddle_movement = STAY;
        wait_for_dist(epsilon, "Mine");
        paddle_movement = EXTEND;
        sleep(10); //???
        paddle_onoff = STOP;
        paddle_movement = STAY;

        //Move to deposit
        //Align to center of arena
        x = 0;
        y = 297;
        //epsilon = 0.2*y;
        set_goal(x, y, -1,"Move to deposit");
        wait_for_dist(epsilon,"Move to deposit");

        //Move up to bin
        x = 0;
        y = -50;
        //epsilon = 100;
        set_goal(x, y, -1,"Move up to bin");
        wait_for_dist(epsilon, "Move up to bin");

        //Now just move straight back until we reack the collection bin
        std::cout<<"\033[0;35m"<< "PATHPLAN: approaching into bin" <<"\033[0m\n";
        control_direction = BACKWARDS;
        sleep(10); //???
        
        //Deposit
        std::cout<<"\033[0;35m"<< "PATHPLAN: deposit " <<"\033[0m\n";
        bin_movement = EXTEND;
        sleep(15); //~15s
        bin_movement = STAY;
        sleep(5); //???
        bin_movement = RETRACT;
        sleep(10); //~10-15s
        bin_movement = STAY;

        //Increment the iteration
        iter = (iter + 1) % 3;
        std::cout<<"\033[0;35m"<< "PATHPLAN: New iteration "<<iter  <<" of type "<< offset[iter] <<"\033[0m\n";
    }
}
