//Checkboard navigation header

#include <string>

void* init_chessboard_navigation( void * stop_flag_ptr );

int get_desired_webcam();

struct locate_actuator
{
    int collection,bin,webcam;
};

locate_actuator get_desired_actuator();

long int millis();
double fmod2pi(double v);

void* init_kinect_mapping(void * stop_flag);

