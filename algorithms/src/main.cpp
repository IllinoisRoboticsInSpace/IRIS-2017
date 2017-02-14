#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>//pthreads
#include <signal.h>

#include "checkboard_navigation_module.h"
#include "data_structure.hpp"

using namespace std;



//Control C handler
void my_handler(int s) {
    cout << "Caught signal "<< s <<"\n";
    exit(1);
}

volatile bool stop_flag = false;

//Create instance of data
navigation_and_mapping_data D;

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

    pthread_t chessboard_t;
    pthread_t navigation_t;
    pthread_t path_planning_t;
    pthread_t fsm_t;

    
		//pthread_init();
    int chessboard = pthread_create(&chessboard_t, NULL, init_chessboard_navigation, (void*)&stop_flag);
    int navigation = pthread_create(&navigation_t, NULL, init_kinect_mapping, (void*)&stop_flag);
    int path = pthread_create(&path_planning_t, NULL, path_planning, 0);
    int fsm = pthread_create(&fsm_t, NULL, FSM, 0);
    if(navigation || chessboard || path || fsm)
        exit(EXIT_FAILURE);

    while (1)
    {
        sleep(1);
    }

    stop_flag = true;

    pthread_join(chessboard_t, 0);
    pthread_join(navigation_t, 0);
    pthread_join(path_planning_t, 0);
    pthread_join(fsm_t, 0);


}
