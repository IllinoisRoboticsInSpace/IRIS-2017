#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "communication.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
//#include "checkboard_navigation_module.h"
//#include "data_structure.hpp"
//#include <ros/ros.h>
//#include "std_msgs/String.h"
#include <sstream>

using namespace std;

char * motor_port=(char*)"/dev/ttyACM0";

void* communication(void * unused)
{
    while(1)
    {
        int file = -1;
        while(file==-1)

        file= open(motor_port,O_RDWR);

    while(1)
    {
        locate_motor desired = get_desired_motor();
        char c[100];
        sprintf(c, "!G 1 %d_!G 2 %d_/n",desired.motor_left,desired.motor_right);
        write(file, c, strlen(c));


    }
}
}
