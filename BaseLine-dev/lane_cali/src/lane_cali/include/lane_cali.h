#ifndef _LANE_CALI_H
#define _LANE_CALI_H
#include <fstream>
#include <ros/ros.h>
#include <time.h>
#include "Eigen/Dense"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "json/json.h"
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <geometry_msgs/Pose.h>
#include "assit.h"
using namespace std;
using namespace Eigen;

struct Point{
    float x;
    float y;
    float z;
};

//intersectionLaneInfo
struct iLaneInfo{
    int angle;
    int stopLine_;
    string mainLidar_ ;      //belong to mainlidar coordinate or not
    string laneAxis_;
    vector<struct Point> come_array_;
    vector<struct Point> leave_array_;
} ;

class LaneCali{

private:
    ros::NodeHandle nh;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    string sub_topic_;
    string pub_topic_;
    int l1;
    int l2;
    int angle;
    // TODO:
    int N_stop_line;
    int W_stop_line;

    int S_stop_line;
    int E_stop_line;

    Eigen::Matrix4f trans_matrix_;
    map<string,struct iLaneInfo> lanes;
    vector<float> trans_;
public:
    time_t begin_t,end_t;
    // void ReadConfig(string config_path);
    void LidarTransform(vector<float> trans_,struct Point * p);
    void LoadLanePos(string configPath_str,map<string,struct iLaneInfo>* lanes);
    bool Init();  
    uint8_t assign(struct iLaneInfo lane,float pos_x,float pos_y);
    int AssignLaneIdAndSensorId(geometry_msgs::Pose obj_pose);//,int &roadID);
    void CallBack(const autoware_msgs::DetectedObjectArray::ConstPtr& msg);
    void Run();


};
#endif
