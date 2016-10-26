#define cos30 0.866025404
#define sin30 0.5

/**GENERICS C++**/
#include <iostream> //cout
#include <string.h>//strcpy
#include <vector> //for std::vector
#include <signal.h>//sighandler
using namespace std;

/**ROS**/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

/**IRIS CODE**/
#include "CoordSystemKinect.hpp"//Kinect Input
#include "libfreenect.hpp"//Kinect Input
#include "Linear.hpp"//Mat3
#include "Map.hpp"//Map<T>

/**KINECT**/
int numMaps;//how many maps do we use per publish(to remove bad data)
bool onOdroid;
bool useOpenGL;
int maxViewDist;//millimeters
int minViewDist;//millimeters
double cellSizeMillis;
int gradientHalfSizeX;
int gradientHalfSizeY;
int minObstacleGroup;
double cellStepTolerance; //fraction of a cells size that a cell must change in height to be marked as steep
int sizeGradientMap;

/**ROS**/
string topicName; //this is the name the listener will look for
const string myNodeName = "iris_obstacles_talker";
unsigned int seq = 0;
string frame_id;
ros::Publisher publisher;

/**FOR CALLBACK**/
int gradientIterator;
std::vector<Map<float> > gradList; 

void callback(const pcl::PCLPointCloud2ConstPtr& input)
{
  Map<float> tempGrad(Vec2i(gradientHalfSizeX, gradientHalfSizeY));
  
//rotation.x1 = cos30;
//rotation.x2 = 0;
//rotation.x3 = -sin30;
//rotation.y1 = 0;
//rotation.y2 = 1;
//rotation.y3 = 0;
//rotation.z1 = sin30;
//rotation.z2 = 0;
//rotation.z3 = cos30;

  if(gradientIterator == numMaps)
    gradientIterator = 0;

  gradList[gradientIterator].clear();
  tempGrad.clear();

  const int pointCount = csk::dimX*csk::dimY;

  Map<float> height(Vec2i(gradientHalfSizeX, gradientHalfSizeY));

  height.getPoint(Vec2i(0,0)).value = 9;
  vector<Vec3f> pointCloud;
  pointCloud.resize(csk::dimX*csk::dimY); //make our pointcloud large enough

//// need to copy data from image->data to a new vector
//short unsigned int * imageData = new short unsigned int[pointCount];

///**REMOVE INVALID POINTS FROM DEPTH DATA**/
//for(int i = 0; i < pointCount; ++i)
//{
//  int milli = csk::RawDepthToMilli(image->data[i]);
//  if(milli < minViewDist || milli > maxViewDist)
//    imageData[i] = 0;
//  else
//    imageData[i] = image->data[i];
//}

///**CREATE CARTESIAN POINT CLOUD**/
//for(int y = 0; y<csk::dimY; ++y)
//{
//  for(int x = 0; x<csk::dimX; ++x)
//  {
//     if(imageData[csk::GetCoord(x,y)] != 0)
//       pointCloud[csk::GetCoord(x,y)] = csk::GetCartCoord(x, y, imageData);
//       ROS_INFO("[%d %d %d]", pointCloud[csk::GetCoord(x,y)].x,pointCloud[csk::GetCoord(x,y)].y,pointCloud[csk::GetCoord(x,y)].z);
//  }
//}
//delete[] imageData;
//pcl::PCLPointCloud2 pcl_pc2;
//pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*input, *temp_cloud);

  
  /**POINT CLOUD ADJUSTED FOR PITCH AND ROLL**/
  for(int i = 0; i < pointCount; ++i)
  {
    pointCloud[i].x = temp_cloud->points[i].z;
    pointCloud[i].y = -temp_cloud->points[i].x;
    pointCloud[i].z = -temp_cloud->points[i].y;
    if (pointCloud[i].x != pointCloud[i].x || 
        pointCloud[i].z > maxViewDist / 1000. || 
        pointCloud[i].z < minViewDist / 1000.)
    {
      pointCloud[i] = Vec3f(0.0, 0.0, 0.0);
    }
//  pointCloud[i] = rotation * pointCloud[i];
    pointCloud[i].x = 0.866025404 * pointCloud[i].x + 0.5 * pointCloud[i].z;
    pointCloud[i].z = -0.5 * pointCloud[i].x + 0.866025404 * pointCloud[i].y;
  }

  /**POINT CLOUD UNITS ADJUSTED FOR HUMAN VIEWING**/
  //half decimeters (50 times larger than a millimeter is half a decimeter)
  //this also determines the representative size of the cells in the map
//for(int i = 0; i<pointCount; ++i)
//{
//  pointCloud[i].z *= 1. / cellSizeMillis;
//  if(onOdroid)
//    pointCloud[i].y *= -1. / cellSizeMillis;/**IF ON ODROID, FLIP IT**/
//  else
//    pointCloud[i].y *= 1. / cellSizeMillis;

//  pointCloud[i].x *= 1. / cellSizeMillis;
//}

  /**CONVERT POINT CLOUD INTO HEIGHT MAP**/
  for(int i = 0; i<pointCount; ++i)
  {
    if(height.getPoint(Vec2i(pointCloud[i].x, pointCloud[i].y)).value < pointCloud[i].z)
          height.getPoint(Vec2i(pointCloud[i].x, pointCloud[i].y)).value = pointCloud[i].z;
  }
  /**REMOVE STRANGE VALUES FROM MAP (filter stuff)**/
  height.makeGradient(gradList[gradientIterator], cellStepTolerance);//tolerance
  height.makeGradient(tempGrad, cellStepTolerance);///SHOULD USE COPY FIX ME HELP DEBUG
  
  /**SETUP DATA STRUCTURES*/
  std::vector<Vec3f> obstacleList;
  sensor_msgs::PointCloud2 rosPointCloud;
  pcl::PointCloud<pcl::PointXYZ> pclPointCloud;

  /**AND TOGETHER GRADIENTS and APPLY FILTER**/
  for(int i=0; i<numMaps; ++i)
      tempGrad.andTogether(gradList[i]);

  tempGrad.filterNum(minObstacleGroup);

  /**GET OBSTACLES**/
  tempGrad.getData(obstacleList);
  const int numObstacles = obstacleList.size();

  /**RESIZE CLOUDS**/
  pclPointCloud.resize(numObstacles);
  rosPointCloud.width = numObstacles;
  rosPointCloud.height = 1;
  rosPointCloud.data.resize(rosPointCloud.width*rosPointCloud.height);

  /**CONVERT AND SET PCL CLOUD**/
  const double gridSizeMeters = cellSizeMillis/1000.;//convert back to meters
  for(int i=0; i<numObstacles; ++i)
  {
    obstacleList[i].x *= gridSizeMeters;
    obstacleList[i].y *= gridSizeMeters;
    obstacleList[i].z *= gridSizeMeters;

    pclPointCloud[i].x = (obstacleList[i].x);
    pclPointCloud[i].y = (obstacleList[i].y);
    pclPointCloud[i].z = (obstacleList[i].z);
  }

  /**PUT PCL CLOUD INTO ROS CLOUD**/
  toROSMsg(pclPointCloud, rosPointCloud);

  /**SET HEADERS FOR ROS**///so we can view in rvis and such
  rosPointCloud.header.seq = ++seq;
  rosPointCloud.header.frame_id = frame_id;
  rosPointCloud.header.stamp = ros::Time::now();

  /**PUBLISH**/
  publisher.publish(rosPointCloud);
  ROS_INFO("I published %d obstacles!", numObstacles);
}

int main(int argc, char **argv)
{
  /**ROS**/
  int bufferSize = 20;
  ros::init(argc, argv, myNodeName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  nh_.param<bool>("flags/onOdroid", onOdroid, false);
  nh_.param<bool>("flags/useOpenGL", useOpenGL, true);
  nh_.param<string>("frame_id", frame_id, "0");
  nh_.param<string>("topic", topicName, "/IRIS/obstacles");
  nh_.param<int>("grid/num_maps", numMaps, 4);
  nh_.param<int>("grid/min_surrounding_obstacles", minObstacleGroup, 2);
  nh_.param<int>("grid/max_view_dist", maxViewDist, 3000);
  nh_.param<int>("grid/min_view_dist", minViewDist, 470);
  nh_.param<double>("grid/cell_size_millis", cellSizeMillis, 100.);
  nh_.param<double>("grid/step_tolerance", cellStepTolerance, 0.5);
  gradientHalfSizeX = int(maxViewDist / cellSizeMillis) + 5;
  gradientHalfSizeY = int(maxViewDist / cellSizeMillis) + 5;
  publisher = nh.advertise<sensor_msgs::PointCloud2>(topicName, bufferSize);
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);
  sizeGradientMap = sizeof(int8_t)*((gradientHalfSizeX*2)+1)*((gradientHalfSizeY*2)+1);

  for(int i = 0; i < numMaps; ++i)
  {
    gradList.push_back(Map<float>(Vec2i(gradientHalfSizeX, gradientHalfSizeY)));
  }

  ros::spin();

  return 0;
  
}

  
