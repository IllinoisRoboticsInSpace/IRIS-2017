// KINECT OBSTACLE DETECTION MODULE
// andres.r.reina@gmail.com
// leonfrickensmith@gmail.com **/
// IRIS at UIUC 2015 **/
// File reuses & inspires in code from the OpenKinect Project. http://www.openkinect.org


/**GENERICS C++**/
#include <iostream> //cout
#include <pthread.h>//pthreads
#include <string.h>//strcpy
#include <vector> //for std::vector
using namespace std;
/**ROS**/
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
/**IRIS CODE**/
#include "CoordSystemKinect.hpp"//Kinect Input
#include "libfreenect.hpp"//Kinect Input
#include "Linear.hpp"//Mat3
#include "data_structure.hpp"
/**OPENGL**/
/*#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>*/
// SERIAL
#include <serial/serial.h>
#include "checkboard_navigation_module.h"
#include "debug_ip_server.h"




void* thread_display(void* arg);


/**KINECT**/
const int maxViewDist = 2500;//millimeters
const int minViewDist = 470;//millimeters
const int gradientHalfSizeX = 80;
const int gradientHalfSizeY = 80;
const int historicHalfSizeX = 80;
const int historicSizeY = 180;
int sizeHTTPimage =0;
const int sizeGradientMap = sizeof(int8_t)*((gradientHalfSizeX*2)+1)*((gradientHalfSizeY*2)+1);
//csk namespace represents CoordinateSystemKinect
const int sizeDepth = FREENECT_DEPTH_11BIT_SIZE;//we need this much space to represent the depth data
const int sizeVideo = FREENECT_VIDEO_RGB_SIZE;//we need this much for the video data
/**ROS**/
const string topicName = "iris_obstacles";//this is the name the listener will look for
const string myNodeName = "iris_obstacles_talker";
/**FOR THREADS**/
volatile bool depth_used = true;
volatile bool video_used = true;
volatile bool depth_displayed = true;
volatile bool map_displayed = true;
volatile bool tcpip_map_used = true;
volatile bool main_stop = false;
volatile bool threads_stop = false;
volatile int got_data_kinect = true;
volatile bool threads_stop_depth = false;
//we can't just use a mutex because the whole purpose is to not block!

template<typename T> T pow2(T x){return x*x;}
double distS(double a){return min(fmod2pi(a),M_PI-fmod2pi(a));}

/**DATA**/
uint16_t* pDepth = NULL;
//char* pVideo = NULL;
//char* pDepthFeed = NULL;
//uint16_t* pDepthDisplay = NULL;
unsigned char* pMapHTTP = NULL;

Vec3f downDirection(0,0,0);//static to prevent other files from seeing this

/**LFN**/
freenect_context* f_ctx=0;
freenect_device* f_dev;


/**======================**/
/**TELLS US THE STEEPNESS FACTOR OF A PIECE OF TERRAIN BY COMPARING THE HEIGHT OF ADJACENT PIECES**/
/**======================**/
template <typename T>
int f_isSteep(T origin, T up, T left, T down, T right, T tolerance)//written by Max Archer 9/17/2014
{
    if(origin!=map_defaultValue)
    {
        if(up==map_defaultValue)
            up = origin;
        if(left==map_defaultValue)
            left = origin;
        if(down==map_defaultValue)
            down = origin;
        if(right==map_defaultValue)
            right = origin;

        if(up-origin >= tolerance || origin-up >= tolerance)
            return map_occupied;
        else if(left-origin >= tolerance || origin-left >= tolerance)
            return map_occupied;
        else if(down-origin >= tolerance || origin-down >= tolerance)
            return map_occupied;
        else if(right-origin >= tolerance || origin-right >= tolerance)
            return map_occupied;
    }
    return map_unoccupied;
}

/**======================**/
/**Uses the input map to produce a gradient of this map**/
/**======================**/
void makeGradient(MATRIX & output, const MATRIX& input, const float tolerance)//takes a map and gives it the gradient data
{
    
    //const float tolerance = 0.5f;
    for(int y = input.yllim()+1; y < input.yhlim()-1; ++y)
    {
        for(int x = input.xllim()+1; x < input.xhlim()-1; ++x)
        {
            if(input(x,y) != map_defaultValue)//get the point, and all points around it!
                output(x,y) = f_isSteep(input(x  , y  ),
                                        input(x  , y+1),
                                        input(x  , y-1),
                                        input(x-1, y  ),
                                        input(x+1, y  ),
                                        tolerance);
        }
    }
}

/**================================================================================**/
/**DEPTH SENSOR CALLBACK**/
/**================================================================================**/
void depth_cb(freenect_device* pDevice, void* v_depth, uint32_t timestamp)
{
    got_data_kinect = true;
    //cout<<"data at depth!\n";
    if(depth_used)
    {
        memcpy(pDepth, v_depth, sizeDepth);
        depth_used = false;
    }
    //if(depth_displayed)
    //{
        //memcpy(pDepthDisplay, v_depth, sizeDepth);
        //depth_displayed = false;
    //}
}
/**================================================================================**/
/**RGB SENSOR CALLBACK**/
/**================================================================================**/
//void video_cb(freenect_device* pDevice, void* v_video, uint32_t timestamp)
//{
    ////cout<<"data at video!\n size_video = "<<sizeVideo<<"\n";
    //if(video_used)
    //{
        //memcpy(pVideo, v_video, sizeVideo);
        //video_used = false;
    //}
//}

float stof0(const string &a) {
        stringstream ss(a);
        float ans;
        ss >> ans;
        return ans;
}
//////////////////////////////////////////////////////////////////////////////////////
// SERIAL INTERPRETER FOR ROLL, PITCH AND YAW  //(Not currently used)
//////////////////////////////////////////////////////////////////////////////////////
Vec3f GetSerialGyro(serial::Serial & s)
{
        std::string buffer;
        static std::string a[3],prev1,prev2,prev3;
        static int message_count=0;
        static Vec3f return_val;
        while(s.available()>5)
        {
                s.readline(buffer,200,"\n");
                if(buffer=="GO\n")
                {
                        if(message_count==4)
                        {
                                prev1=a[0];
                                prev2=a[1];
                                prev3=a[2];
                                //ROS_INFO("Received IMU data");
                        }
                        message_count=0;                
                }
                if(message_count!=0 && message_count<4)
                        a[message_count-1]=buffer;
                buffer="";
                message_count++;        
        }
        if(prev1!="" || prev2!="" || prev3!="" ) 
        {
                return_val=Vec3f(stof0(prev3)*M_PI/180,stof0(prev2)*M_PI/180,stof0(prev1)*M_PI/180);
                //cout << return_val.z;
        }
        else
                ROS_INFO("*** No IMU data in last loop");
        return return_val;      
}

// SERIAL INITIALIZATION
bool  SerialConnect(serial::Serial & ser)
{
        try
        {
                ser.setPort("/dev/ttyACM0");
                ser.setBaudrate(9600);//115200
                serial::Timeout to = serial::Timeout::simpleTimeout(1);
                ser.setTimeout(to);
                ser.open();
        }
        catch (serial::IOException& e)
        {
                ROS_ERROR_STREAM("Unable to open port ");
                ROS_INFO("Unable to open port ");
                perror("Unable to open port ");
        }

        if (ser.isOpen()) {
                ROS_INFO_STREAM("Serial Port initialized");
                return true;
        }
        else {
                ROS_ERROR_STREAM("Unable to open port ");
                ROS_INFO("Unable to open port ");
                perror("Unable to open port ");
        }
        return false;
}

/**================================================================================**/
/**DEPTH PROCESS THREAD**/
/**================================================================================**/
void* thread_depth(void* arg)
{
    MATRIX historic(-historicHalfSizeX,historicHalfSizeX, 0, historicSizeY);
    MATRIX gradient(-gradientHalfSizeX,gradientHalfSizeX, -gradientHalfSizeY,gradientHalfSizeY);
    MATRIX height(-gradientHalfSizeX,gradientHalfSizeX, -gradientHalfSizeY,gradientHalfSizeY);
    D.map=&historic;
    D.local_map=&gradient;
    
    historic.fill(map_defaultValue);

    while(not threads_stop)
    {
        if(not depth_used && pDepth != NULL)//make sure we don't take an image with bad accelerometer data
        {
            if(downDirection.z == 0)
                ROS_INFO("\nNo Data From Kinect Accelerometer!");

            const int pointCount = csk::dimX*csk::dimY;
            gradient.fill(map_defaultValue);
            height.fill(map_defaultValue);

            vector<Vec3f> pointCloud;
            pointCloud.resize(csk::dimX*csk::dimY);//make our pointcloud large enough

            /**REMOVE INVALID POINTS FROM DEPTH DATA**/
            for(int i = 0; i<pointCount; ++i)
            {
                int milli = csk::RawDepthToMilli(pDepth[i]);
                if(milli < minViewDist || milli > maxViewDist)
                    pDepth[i] = 0;
            }
            /**CREATE CARTESIAN POINT CLOUD**/
            for(int y = 0; y<csk::dimY; ++y)
                for(int x = 0; x<csk::dimX; ++x)
                {
                    if(pDepth[csk::GetIndex(x,y)] != 0)
                        pointCloud[csk::GetIndex(x,y)] = csk::GetCartCoord(x, y, pDepth);
                }
                
            depth_used = true;
                                            
            // GET YAW ANGLE FROM SERIAL
            chesspos robot_pos = get_chessboard_navigation_pos();
            
            /**POINT CLOUD ADJUSTED FOR PITCH, ROLL AND YAW**/
            Mat3f pitchRoll = csk::FindDownMatrix(downDirection,robot_pos.t);//find the rotation matrix
            for(int i = 0; i<pointCount; ++i)//rotate the point cloud data appropriatly
            {
                pointCloud[i] = pitchRoll*pointCloud[i];
            }
            /**POINT CLOUD UNITS ADJUSTED FOR HUMAN VIEWING**/
            const float unitConvert = 1.0f/50.0f;//half decimeters (50 times larger than a millimeter is half a decimeter)
            //this also determines the representative size of the cells in the map
            for(int i = 0; i<pointCount; ++i)
            {
                pointCloud[i].z *= unitConvert;
                pointCloud[i].y *= unitConvert;
                pointCloud[i].x *= unitConvert;
            }
            /**CONVERT POINT CLOUD INTO HEIGHT MAP**/
            for(int i = 0; i<pointCount; ++i)
            {
                if(height(pointCloud[i].x, pointCloud[i].y) < pointCloud[i].z)
                    height(pointCloud[i].x, pointCloud[i].y) = pointCloud[i].z;
            }
            /**REMOVE STRANGE VALUES FROM MAP**/
            const float cellStepTolerance = 0.5;//fraction of a cells size that a cell
            //can change in height and will be marked as steep afterward
            makeGradient(gradient, height, cellStepTolerance);//tolerance

            int xPos=robot_pos.x/5; //position of the robot (true one)
            int yPos=robot_pos.y/5;

            if(millis()-robot_pos.millis<1000)
            {
                //PROJECT COMPUTED GRADIENT INTO HISTORIC MAP //
                for(int x_i =-gradientHalfSizeX ; x_i < gradientHalfSizeX; x_i++)
                {
                    for( int y_i = -gradientHalfSizeY ; y_i < gradientHalfSizeY ; y_i++)
                    {
                        if(gradient(x_i,y_i) != map_defaultValue &&
                                        x_i+xPos>=-historicHalfSizeX && y_i+yPos>=0 &&
                                        x_i+xPos < historicHalfSizeX && y_i+yPos < historicSizeY)
                            historic(x_i+xPos,y_i+yPos) = gradient(x_i,y_i);
                    }
                }
            }
            
            if(tcpip_map_used)
            {
                for(int i=0; i<sizeHTTPimage; i+=3)
                {
                    int x = i/3;
                    int px =x%(historicHalfSizeX*2)-historicHalfSizeX;
                    int py =historicSizeY-x/(historicHalfSizeX*2);
                    float val = historic( px,py );
                    if(px==0 || py==0) //Mark axis
                    {
                        pMapHTTP[i+0] = 0;
                        pMapHTTP[i+1] = 255;
                        pMapHTTP[i+2] = 0;
                    }
                    else if(pow2(xPos-px)+pow2(py-yPos)<(distS(atan2(1.*py-yPos,1.*xPos-px)-robot_pos.t)<2?30:5)) //Mark current position
                    {
                        pMapHTTP[i+0] = 0;
                        pMapHTTP[i+1] = 0;
                        pMapHTTP[i+2] = 255;
                    }
                    else if(val == map_defaultValue)
                    {
                        pMapHTTP[i+0] = 0;//red
                        pMapHTTP[i+1] = 0;//green
                        pMapHTTP[i+2] = 0;//blue
                    }
                    else if(val == 1)//it is an obstacle
                    {
                        pMapHTTP[i+0] = 255;
                        pMapHTTP[i+1] = 0;
                        pMapHTTP[i+2] = 0;
                    }
                    else
                    {
                        pMapHTTP[i+0] = 255;
                        pMapHTTP[i+1] = 255;
                        pMapHTTP[i+2] = 255;
                    }
                }
                tcpip_map_used = false;
            }
            
        }
        else
            usleep(1);//if we can't do stuff, just give control back to the processor!
    }
    
    return NULL;
}


/**================================================================================**/
/**KINECT UPDATE THREAD**/
/**================================================================================**/
void* thread_kinect(void* arg)
{
    /**MISC KINECT COMMANDS**/
    //freenect_set_tilt_degs(f_dev, -22);//set kinect angle
    //freenect_set_led(f_dev, static_cast<LED_COLOR>(3));//set kinect LED color, LED_RED, libfreenect.h

    /**SETUP VIDEO**/
    //freenect_set_video_callback(f_dev, video_cb);
    freenect_set_video_format(f_dev, FREENECT_VIDEO_RGB);
    freenect_start_video(f_dev);//tell it to start reading rgb

    /**SETUP DEPTH**/
    freenect_set_depth_callback(f_dev, depth_cb);//set the function that will be called for each depth call
    freenect_set_depth_format(f_dev, FREENECT_DEPTH_11BIT);
    freenect_start_depth(f_dev);//tell it to start reading depth


    while(!threads_stop_depth && !threads_stop && freenect_process_events(f_ctx) >= 0)/**this is primary loop for kinect stuff**/
    {
        double dx,dy,dz;
        freenect_raw_tilt_state* pState;
        freenect_update_tilt_state(f_dev);
        pState = freenect_get_tilt_state(f_dev);
        freenect_get_mks_accel(pState, &dx, &dy, &dz);
        downDirection = csk::FindDown(pState->accelerometer_x, pState->accelerometer_y, pState->accelerometer_z);
        //cout << "\nDown:\t" << downDirection.x << "\t" << downDirection.y << "\t" << downDirection.z;
        //usleep(10000);
    }

    /**SHUT DOWN STREAMS**/
    freenect_stop_video(f_dev);
    freenect_stop_depth(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    return NULL;
}


/**================================================================================**/
/**=================================  MAIN  =======================================**/
/**================================================================================**/
void* init_kinect_mapping(void * stop_flag)
{
        volatile bool* async_stop_flag = (bool*)stop_flag;
        /**===================================================**/
        /**ALL ABOUT INITIALIZING THE CONNECTION WITH KINECT!!**/
        /**===================================================**/
        //pDepthDisplay = static_cast<uint16_t*>(malloc(sizeDepth));
        //pDepthFeed = static_cast<char*>(malloc(sizeVideo));//used to rgb display what the kinect sees
        pDepth = static_cast<uint16_t*>(malloc(sizeDepth));//each point is a uint16_t for depth
        //pVideo = static_cast<char*>(malloc(sizeVideo));//each point needs 3 chars to represent the color there (r255,g255,b255)

        sizeHTTPimage = historicHalfSizeX * historicSizeY * 2 * 3;
        pMapHTTP = static_cast<unsigned char*>(malloc(sizeHTTPimage)); //http map buffer

        threads_stop = false;
        debug_ip_server(8080, &threads_stop, &tcpip_map_used, pMapHTTP, sizeHTTPimage, historicHalfSizeX * 2, historicSizeY );

        pthread_t depth_t;

        int user_device_number = 0;
        while (!(*async_stop_flag))
        {
                while (!(*async_stop_flag))
                {
                        if (freenect_init(&f_ctx, NULL) < 0)
                        {
                                cout << "Freenect_init() failed.(1)\n";
                                return false;
                        }
                        freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

                        while (!(*async_stop_flag))
                        {
                                int nr_devices = freenect_num_devices(f_ctx);
                                cout << "Number of KINECT devices found: " << nr_devices<<"\n";

                                if (nr_devices < 1)
                                        cout << "KINECT No devices found.(2)\n";
                                else
                                        break;
                                sleep(2);
                        }
                        if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0)
                        {
                                cout << "KINECT Could not open device.(3)\n";
                        }
                        else
                        {
                                cout << "KINECT Opened a device.\n";
                                break;
                        }
                        user_device_number++;
                        if (user_device_number > 6)
                        {
                            user_device_number = 0;
                            sleep(2);
                        }
                        freenect_shutdown(f_ctx);
                        cout << "Freenect KINECT initialization failed. Trying device " << user_device_number << "\n";
                }

                if (!(*async_stop_flag))
                {
                        /**THREADS TO SIMULTANEOUSLY RUN THE SENSOR INPUT AND COMPUTATION**/
                        pthread_t kinect_t;
                        threads_stop_depth = false;
                        int kinect = pthread_create(&kinect_t, NULL, thread_kinect, NULL);
                        int map = pthread_create(&depth_t, NULL, thread_depth, NULL);
                        /**MAKE SURE THEY WERE CREATED**/
                        if (kinect || map)
                        {
                                cout << "KINECT PThread_create failed.(5)\n";
                                return false;
                        }

                        while (!(*async_stop_flag))
                        {
                                //need watchdog
                                got_data_kinect = false;
                                sleep(5);
                                if(!got_data_kinect)
                                {
                                    cout << "KINECT ERROR watchdog is triggering restart\n";
                                    break; //restart all
                                }
                        }

                        threads_stop_depth = true;
                        pthread_join(kinect_t, 0);
                        threads_stop_depth = false;

                        freenect_shutdown(f_ctx);
                }
        }
        threads_stop = true;
        pthread_join(depth_t, 0);

    //free(pDepthDisplay);
    //free(pDepthFeed);
    free(pDepth);
    //free(pVideo);
    free(pMapHTTP);

    return 0;
}
