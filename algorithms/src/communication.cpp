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
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "checkboard_navigation_module.h"
//#include "data_structure.hpp"
//#include <ros/ros.h>
//#include "std_msgs/String.h"

using namespace std;

void* communication_motor(void * unused)
{
    printf("COMMUNICATION: start motor");
    tcp_send motor_serial((char*)"localhost",9002);
    while(1)
    {
        locate_motor desired = get_desired_motor();
        char c[100];
        sprintf(c, "!G 1 %d_!G 2 %d_\n",desired.motor_left,desired.motor_right);
        //printf("%s",c);
        motor_serial.send(c, strlen(c));
        sleep(1);

    }
}

void* communication_actuators(void * unused)
{
    printf("COMMUNICATION: start actuators");
    tcp_send actuator_serial((char*)"localhost",9001);
    while(1)
    {
        int desired_angle = get_desired_webcam();
        locate_actuator desired = get_desired_actuator();
        char c[100];
        sprintf(c,"!%d,%d,%d,%d#!\n",desired.collection,desired.bin,desired.webcam,desired_angle);
        //printf("%s",c);
        actuator_serial.send(c, strlen(c));
        sleep(1);

    }
}

tcp_send::tcp_send(char* _address, int _port){
    address = _address;
    port = _port;
    sock = 0;

}
int tcp_send::send(char* data, int size){
    if(size <= 0){
        return 0;
    }
    while(1){

        tcp_send::internal_reconnect();

        int n = write(sock, data, size);
        if(n<=0){
            printf("tcp_send: ERROR write failed %s:%d \n", address,port);
            close(sock);
            sock = -1;
            sleep(1);
            continue;
        }
        else if(n!=size){
            return tcp_send::send(&data[n],size-n) + n;
        }
        return n;
    }
}

int tcp_send::receive(char* data, int size){
    while(1){

        tcp_send::internal_reconnect();

        int n = recv(sock, data, size,0);

        if(n<=0){
            printf("tcp_send: ERROR read failed %s:%d \n", address,port);
            close(sock);
            sock = -1;
            sleep(1);
            continue;
        }
        return n;
    }
}

void tcp_send::internal_reconnect(){
    while(sock<=0){
            sock = socket(AF_INET, SOCK_STREAM,0);
            if(sock==-1){
                printf("tcp_send: ERROR socket failed\n");
                sleep(1);
                continue;
            }
            struct sockaddr_in serv_addr;
            struct hostent *server;
            server = gethostbyname(address);
            bzero((char *) &serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
            serv_addr.sin_port = htons(port);
            int r = connect(sock,(struct sockaddr *)&serv_addr,sizeof(serv_addr));
            if(r==-1){
                printf("tcp_send: ERROR connect failed: %s:%d\n", address,port);
                close(sock);
                sock = -1;
                sleep(1);
                continue;
            }
            printf("tcp_send: success socket connection %s:%d \n", address,port);
        }
}
