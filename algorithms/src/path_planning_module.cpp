#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>


#include "checkboard_navigation_module.h"
#include "data_structure.hpp"
#include "MapTransformer.hpp"
#include "RRT.hpp"
#include "communication.hpp"

//#include <ros/ros.h>
//#include "std_msgs/String.h"
#include <sstream>

using namespace std;
string positionsString;
volatile bool positionStringIsUsed = true;
extern volatile bool pathplan_map_used;

//#define pow2(x) (x)*(x)

const double LINEAR_CONST = 1000/2.;
const double ANGULAR_CONST = 1000/0.05;
const double CURVE_JSON_STEP = 1;
const double POINT1_CURVE_LENGTH = 4;
const double POINT2_CURVE_LENGTH = 8;
const double F_FORWARD_CONTROL = 500;
const double K_TURN_CONTROL = 180;

//Global variables
volatile double goal_x=0.;
volatile double goal_y=0.;
extern MATRIX pathplan_map;

enum{RETRACT = 0, STAY = 1, EXTEND = 2, STOP = 0, MOVE = 1, BACKWARDS = -2};

volatile int bin_movement = 1; // 0=RETRACT 1=STAY 2=EXTEND
volatile int paddle_movement = 1; // 0=RETRACT 1=STAY 2=EXTEND
volatile int paddle_onoff = 0; // 0=STOP 1=MOVE

//minimum path radious for RRT
const double min_radius = 10;

volatile int control_direction=1;
/**
* Checks whether or not if the robot will collide if it is at x,y, and angle theta.
* @return false if the robot will collide, true otherwise.
*/
bool collision_checker_f(double x, double y, double theta){
    int robotCellHalfWidth = 10;
    int robotCellHalfHeight = 10;
    for (int robotX=-robotCellHalfWidth; robotX<=robotCellHalfWidth; ++robotX){
        for (int robotY=-robotCellHalfWidth; robotY<=robotCellHalfWidth; ++robotY){
            Vec2f point(robotX,robotY);
            point = MapTransformer::rotate_point(point,theta);
            point = MapTransformer::translate_point(point,x,y);
            if (pathplan_map.validIndex((int) point.x, (int) point.y) && pathplan_map((int) point.x, (int) point.y)==map_occupied){
                return false;
            }
        }
    }
    return true;
}


//absolute value templated
template<typename T> T absd(T d){return d<0?-d:d;}
//efficient square templated
//template<typename T> T pow2(T d){return d*d;}
//find distance in S(2pi) set (like a circle: through either side)
template<typename T> T diff2pi(T d)
{
    d=fmod2pi(d);
    return min(d,2*M_PI-d);
}

locate_motor desired_motor_action;

void* path_planning(void* unused)
{
    //Initialize ROS node and publisher
    int count_loops=0;
//    ros::NodeHandle n;
//    ros::Publisher pub_control=n.advertise<std_msgs::String>("/IRIS/autonomous_command", 1);
    chesspos poss = {0,0,0,0};
    while(poss.millis==0) //wait for first location
        poss = get_chessboard_navigation_pos();
    
    std::cout<<"\033[0;32m"<< "PATHPLAN: got first position!"<<"\033[0m\n";
    
    while(1)
    {
        double forward_cntl;
        double turning_cntl;
        //Get robot position
        chesspos pos = get_chessboard_navigation_pos();
        
        if(!pathplan_map_used) {
    		

            //Tells where the robot starts and ends at
            pose2d start(pos.x/5., pos.y/5., pos.t+M_PI);
            pose2d end(goal_x/5., goal_y/5., pos.t); //change this later

            //creates a random path generator, runs 500 iterations per round, avoiding obstacles
            //check RRT.hpp to see how this function works
            path p = RRT(path_planner_functions<collision_checker_f_prototype>( min_radius, collision_checker_f), start, end, 500, false);

            pose2d nextGoal;
            //nextGoal is changed in the following function, passed by reference
            if (p.get_position(10, nextGoal)) {
                std::cout<<"\033[0;32m"<< "PATHPLAN: path exists!"<<"\033[0m\n";
            }
            else {
                std::cout<<"\033[0;42m"<< "PATHPLAN: **************** error in the path *******************"<<"\033[0m\n";
            }

            pathplan_map_used = true;            
            
            if (positionStringIsUsed) {
                //This string stores the Json data for the path
                positionsString = "{\"data\":[";
                
                //The length along the curve
                double curveLength = 0.0;
                
                //The distance in centimeters by which curveLength advances each time the following loop iterates
                double step = 1.0;
                
                //while next position is valid
                pose2d currentPoint;
                while (p.get_position(curveLength, currentPoint)) {
					// start line with comma only if not first item
					positionsString += (curveLength==0.0?"[":",[");
					// x
					positionsString += std::to_string(currentPoint.p.x);
					positionsString += ",";
					// y
					positionsString += std::to_string(currentPoint.p.y);
					positionsString += ",";
					// theta
					positionsString += std::to_string(currentPoint.t);
					positionsString += "]";
					// advance in curve length
					curveLength+=CURVE_JSON_STEP;
                }
                
                //finish string
                positionsString += "]}";
                
                // this is the semaphore
                //when false debug_ip_server is controlling positionsString
                positionStringIsUsed = false;
            }
            
            
            
            pose2d point1,point2;
            if(millis()-pos.millis<2500 
                && p.get_position(POINT1_CURVE_LENGTH, point1) 
                && p.get_position(POINT2_CURVE_LENGTH, point2))
            {
            
                double theta=fmod(point2.t-point1.t+M_PI,2*M_PI)-M_PI;
                
                turning_cntl=K_TURN_CONTROL*theta;
                forward_cntl=F_FORWARD_CONTROL;
            }
            else
            {
				std::cout<<"\033[0;32m"<< "PATHPLAN: ********* position is too old ******** "<<"\033[0m\n";
                turning_cntl=0;
                forward_cntl=0;
            }
            
            //normalize and get right and left values
            double normalizer=absd(turning_cntl)+absd(forward_cntl);
            if(normalizer>1000.)
            {
                turning_cntl/=normalizer/1000.;
                forward_cntl/=normalizer/1000.;
            }
            desired_motor_action.motor_right=forward_cntl+turning_cntl;
            desired_motor_action.motor_left=forward_cntl-turning_cntl;
 
            std::cout<<"\033[0;32m"<< "PATHPLAN: current "<<pos.x<<" "<<pos.y<<" "<<pos.t<<" target "<< goal_x << " " << goal_y << " action l "<< left << " r " << right <<"\033[0m\n";
 
        }
        else
        {
            desired_motor_action.motor_right=0;
            desired_motor_action.motor_left=0;
        }
        
        count_loops++;
    }
}

locate_motor get_desired_motor()
{
	return desired_motor_action;
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
