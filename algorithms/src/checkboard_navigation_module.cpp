#include <iostream>
#include <sstream>
#include <time.h>
#include <sys/time.h> 
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <SerialStream.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "checkboard_navigation_module.h"
//0//#include <ros/ros.h>
//0//#include <std_msgs/Float32.h>
#include "data_structure.hpp"
#include "communication.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;
using namespace LibSerial;

#include "claibinit_mod.h"

float lastDelta=1;

//double fmod2pi(double v)
//{
    //return fmod(fmod(v,M_PI*2)+M_PI*4,M_PI*2);
//}

const float angle_break=270;
const float angle_scale=-.5;
const float angle_offset=-270;
const float angle_multiplier=1; //1/3.;
static const int delta_angle = 20;

long int millis()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
}

volatile int webcam_pos_to_serial = 0;

int get_desired_webcam()
{
    return webcam_pos_to_serial;
}

//Global settings file
chesspos pos_chesspos={0.,0.,0.,0};
//volatile int lock=0;

chesspos get_chessboard_navigation_pos()
{
    //while(lock)
    //{
        //cout<<"Process is locked trying to access chessboard navigation data"<<endl;
        //while(lock)
            //usleep(1000);
    //}
    //lock=1;
    chesspos ret;
    ret.x=pos_chesspos.x;
    ret.y=pos_chesspos.y;
    ret.t=pos_chesspos.t;
    ret.millis=pos_chesspos.millis;
    //lock=0;
    return ret;
}


long convertToEpoch(long v4l_ts_ms) {
    //To solve any issues go to:
    //    http://answers.opencv.org/question/61099/is-it-possible-to-get-frame-timestamps-for-live-streaming-video-frames-on-linux/
    static long monotonicToEpochOffset_ms = -1; //<-- initialize in global scope
    if (monotonicToEpochOffset_ms == -1) {
        struct timespec  vsTime;  clock_gettime(CLOCK_MONOTONIC, &vsTime);

        long uptime_ms = vsTime.tv_sec * 1000 + (long)round(vsTime.tv_nsec / 1000000.0);
        long epoch_ms = millis();

        // add this quantity to the CV_CAP_PROP_POS_MEC to get unix time stamped frames
        monotonicToEpochOffset_ms = epoch_ms - uptime_ms;
    }

    return monotonicToEpochOffset_ms + v4l_ts_ms;

}

bool nextImage(VideoCapture & inputCapture, Mat & result, long * millis_timestamp=0)
{
    int count_trials=0;
    for (long i = millis();millis() - i < 20;)//stay 20 ms discarding frames to ensure we get a current frame at the end
    {
        count_trials++;
        long j = millis();
        if (!inputCapture.grab())
        {
            cout << "WEBCAM IMAGE GRAB FAILED eventually trying to restart\n";
            return false;
        }
        if (millis() - j > 10)
        {
            //cout<<"took new image in "<<millis()-j<<" millis\n";
            break;
        }
    }
    if(count_trials>180) //too quick to be really getting pictures, reset camera!
    {
        cout << "WEBCAM IMAGE GRAB IS GOING NUTS - ERROR, eventually trying to restart\n";
        return false;
    }
    //cout<<"have new image \n";
    if (!inputCapture.retrieve(result))
    {
        cout << "WEBCAM IMAGE RETRIEVE FAILED eventually trying to restart\n";
        return false;
    }

    //cout<<"CV_CAP_PROP_POS_MSEC: " << convertToEpoch(inputCapture.get(CV_CAP_PROP_POS_MEC)) <<"\n" <<
          //"CV_CAP_PROP_POS_FRAMES: "<< (long)inputCapture.get(CV_CAP_PROP_POS_FRAMES) << "\n" <<  // <-- the v4l2 'sequence' field
          //"CV_CAP_PROP_FPS:  "<< inputCapture.get(CV_CAP_PROP_FPS) << "\n";

    if (millis_timestamp)*millis_timestamp = millis(); //convertToEpoch(inputCapture.get(CV_CAP_PROP_POS_MEC));

    return true;
}

void* init_chessboard_navigation(void * stop_flag_ptr )
{
    static const int avg_num = 3;
    static float x_hist[avg_num];
    static float y_hist[avg_num];
    static int count=0;
    static int i_hist=0;
    volatile bool * stop_flag = (bool*) stop_flag_ptr;
    //0//ros::NodeHandle n("chessboard_navigation");
    //0//ros::Publisher pub = n.advertise<std_msgs::Float32>("/IRIS/webcam_angle", 1);
    float webcam_angle = 0;
    pos_chesspos.x = 0;
    pos_chesspos.y = 0;
    pos_chesspos.t = 0;
    pos_chesspos.millis = 0;
    Size boardSize(4,3);   //CHANGE HERE THE AMOUNT OF SQUARES
    float depth = 6.03;    //CHANGE HERE THE SHAPE  OF SQUARES
    float squareSize =4.13;//CHANGE HERE THE SHAPE  OF SQUARES

    int camera_id = 1;
    VideoCapture inputCapture;
    
    webcam_pos_to_serial=((angle_offset) * angle_scale);
     
    for (int re_connect_retries = 0;!(*stop_flag);++re_connect_retries)
    {
        cout << "Initializing webcam navigation with device " << camera_id << endl;

        inputCapture.open(camera_id); //OPEN CAMERA camera_id
        inputCapture.set(CV_CAP_PROP_FRAME_WIDTH, 6400);
        inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 4800);
        inputCapture.set(CV_CAP_PROP_FPS, 15);
        if (!inputCapture.isOpened())
        {
            camera_id++;
            if (camera_id > 6)
            {
                sleep(3);
                camera_id = 0;
            }
            usleep(200000);
            cout << "WEBCAM ERROR retrying: ";
            continue; // try next device
        }



        Mat cameraMatrix, distCoeffs;
        Size imageSize;
        clock_t prevTimestamp = 0;
        const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
        const char ESC_KEY = 27;

        cout << "Webcam navigation ready!" << endl;

        int count_lost = 9999;

        for (int i = 0;!(*stop_flag);++i)
        {
            long millis_timestamp;

            Mat view;
            if (!nextImage(inputCapture, view,&millis_timestamp))
                break;
            if (i == 0) cout << "webcam image size is " << view.cols << "x" << view.rows << "\n";
            imshow("Image View", view);
    
            //cout << "Webcam navigation data!" << endl;

            imageSize = view.size();  // Format input image.
            if (imageSize.width == 0)
            {
                cout << "Empty webcam image received :(  *************************" << endl;
                static int count_fail = 10;
                usleep(10000);
                if (count_fail-- <= 0)
                {
                    count_fail = 10;
                    break;
                }
                continue;
            }
            flip(view, view, 0);

            vector<Point2f> pointBuf;

            bool found = findChessboardCorners(view, boardSize,
                    pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

            if (found)                // If done with success,
            {
                count_lost = 0;
                // improve the found corners' coordinate accuracy for chessboard
                //Mat viewGray;
                //cvtColor(view, viewGray, COLOR_BGR2GRAY);
                //cornerSubPix(viewGray, pointBuf, Size(11, 11),
                //    Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                

                // Draw the corners.
                drawChessboardCorners(view, boardSize, Mat(pointBuf), found);

                //cout << endl<<endl<< "***************** list *******************" << endl;

                //if (boardSize.width == 4 && boardSize.height == 3)
                {
                    const int Alist[15][2]={
                        { 0, 1},{ 1, 2},{ 2, 3},
                        { 4, 5},{ 5, 6},{ 6, 7},
                        { 8, 9},{ 9,10},{10,11},
                        { 4, 9},{ 5,10},{ 6,11},
                        { 8, 5},{ 9, 6},{10, 7}
                        };
                    const int Blist[8][2]={
                        { 0, 4},{ 1, 5},{ 2, 6},{ 3, 7},
                        { 0, 8},{ 1, 9},{ 2,10},{ 3,11}
                        };
                    float a = 0;
                    float b = 0;
                    float c = 0;
                    float& w = squareSize;
                    float& d = depth;
                    for (int ii = 0;ii < 15;ii++)
                        a += pointBuf[Alist[ii][0]].x - pointBuf[Alist[ii][1]].x;
                    for (int ii = 0;ii < 8;ii++)
                        b += pointBuf[Blist[ii][0]].x - pointBuf[Blist[ii][1]].x;
                    for (int ii = 0;ii < 12;ii++)
                        c += pointBuf[ii].x;
                    a /= 15;
                    b /= 8;
                    c /= 12;
                    if (a < 0)
                    {
                        a = -a;
                        b = -b;
                    }
                    b = -b;
                    //find circles center and radious
                    //for(vector<Point2f>::iterator it=pointBuf.begin();it!=pointBuf.end();it++)
                    //{
                    //    cout << "Point " << it->x << " , " << it->y << endl;
                    //}
                    //cout << "a " << a/50 << " b " << b/50 << endl;
                    a *= 2.226618*0.6920/ view.cols;
                    b *= 2.226618 / view.cols;
                    c -= view.cols / 2;
                    c *= 2.226618 / view.cols;
                    float dx = w + d*tan(M_PI / 2 - b);
                    float dy = d + w*tan(M_PI / 2 - a);
                    float dperp = (w*dy - (dy * 2 - d)*dx) / (dx*dx + dy*dy);
                    float x = w + dy*dperp;
                    float y = d - dx*dperp;
                    x_hist[i_hist]=x;
                    y_hist[i_hist]=y;
                    i_hist=(i_hist+1)%avg_num;
                    count ++;
                    float avgx=0,avgy=0;
                    for(i=0;i<min(count,avg_num);i++)
                    {
                        avgx+=x_hist[i];
                        avgy+=y_hist[i];
                    }
                    avgx/=min(count,avg_num);
                    avgy/=min(count,avg_num);
                    x=avgx;
                    y=avgy;
                    //cout << "dx " << dx << " dy " << dy << endl;

                    //rotate webcam!
                    float delta = c*180. / M_PI * angle_multiplier ;
                    if (delta!=0){
                        lastDelta=delta;
                        printf("THIS IS DELTA YO %f\n", lastDelta);
                    }
                    float actual_webcam_angle=webcam_angle+c*180. / M_PI
                    if (abs(delta) > 1)
                        webcam_angle += delta;
                    bool long_turn = false;
                    if (webcam_angle < (angle_break-360+5))
                    {
                        webcam_angle = angle_break-5;
                        long_turn = true;
                    }
                    if (webcam_angle > (angle_break-5))
                    {
                        webcam_angle = (angle_break-360+5);
                        long_turn = true;
                    }

                    webcam_pos_to_serial=(int)((angle_offset+webcam_angle) * angle_scale);
                    printf("Angle sent:%d = %d in the serial\n", (int)webcam_angle, (int)((angle_offset+webcam_angle) * angle_scale));
                    
                    double vehicle_angle = -fmod2pi((actual_webcam_angle-90-20)*M_PI / 180. - atan2(y, x) - M_PI)+M_PI*2.;

                    cout << "webcam nav " << "x(cm) " << x << " y(cm) " << y << " "
                         << "x(in) " << x/2.54 << " y(in) " << y/2.54 << endl
                         << "webcam angle command th " << webcam_angle << " actual webcam angle "<<actual_webcam_angle 
                         << " delta " << delta << " vehicle \e[35m" << vehicle_angle*180. / M_PI << "\e[0m" <<endl;

                    //while (lock);
                    //lock = 1;
                    pos_chesspos.x = x;
                    pos_chesspos.y = y;
                    pos_chesspos.t = vehicle_angle;
                    pos_chesspos.millis = millis_timestamp;
                    //lock = 0;

                    if (long_turn || abs(delta) > 1)
                    {
                        long int t = millis();
                        while (millis() - t < (long_turn ? 3000 : 200))
                             nextImage(inputCapture, view);
                    }

                }
                //else cout << "Wrong pattern size!";
            }
            else
            {
                count_lost++;
                if (count_lost > 10)
                {
                    static int sweep_dir;
                    cout << "WEBCAM: too long lost, doing 180s\n";
                    if (lastDelta!=0){
                        sweep_dir = (int)(lastDelta/abs(lastDelta));
                    }else{
                        sweep_dir = 1;
                    }
                    printf("THIS IS DELTA YO %f\n", lastDelta);

                    if (webcam_angle + delta_angle*sweep_dir > angle_break || webcam_angle + delta_angle*sweep_dir < (angle_break-360)){
                        sweep_dir = -sweep_dir;
                        lastDelta = sweep_dir;
                    }
                    webcam_angle += delta_angle*sweep_dir;
                    
                    webcam_pos_to_serial=(int)((angle_offset+webcam_angle) * angle_scale);
                    printf("Angle sent:%d = %d in the serial\n", (int)webcam_angle, (int)((angle_offset+webcam_angle) * angle_scale));
                    
                    long int t = millis();
                    while (millis() - t < 200)
                         nextImage(inputCapture, view);
                    count_lost = 8;
                }
                else
                    cout << "WEBCAM: pattern lost\n";
            }

        }

        cout << "WEBCAM ERROR - Lost connection to camera!";

    }
    cout << "*** Webcam process finished";

    return 0;
}


