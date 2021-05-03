#ifndef  __CONVERT_H__
#define  __CONVERT_H__

#include <ros/ros.h>
#include <sys/time.h>
#include <signal.h>
#include <csignal>
#include "radar_msgs/RadarObjects_General.h"
#include "radar_msgs/RadarObjectsArray.h"
#include "radar_msgs/RadarPosition.h"

#include "mec_radar.h"

using namespace std;

bool app_stopped = false;
int radar_num =0 ;
int port = 0;
string param_dir = "/home";
string adv_name ="radar_";
vector<ros::Publisher> pub;
int same_direction = 0;
vector <struct radar_info> radar_info;

struct  radar_info
{  
    std::string radar_ip;  
    // std::string device_id;
    int vehicle_stop_line ;
    int       close_direction_lane_num ;
    int       away_direction_lane_num ;            
    float    *close_lane ;         
    float    *away_lane ;//= new float[away_direction_lane_num+1];               
};

map<string,int> direction_roadID_map = {
    {"NN",1},
    {"EE",2},
    {"SS",3},
    {"WW",4},
    {"NW",1},
    {"NE",2},
    {"SE",3},
    {"SW",4},
};

void sig_handler( int sig );

bool init(int argc, char **argv);

float* split_function(std::string input_str,const char *split_signal,int elements_num);

vector<struct radar_info> loadRadarInfo(string param_dir);

static void obj_data_convert(const TObjectData* ptObjTmp,
                            radar_msgs::RadarObjects_General & radar_obj_general,int channel,struct radar_info radar_info);

static void dump_rt_data(TRadarData* ptData,ros::Publisher radar_pub,int channel,struct radar_info radar_info);//传入ptData[i]


void* get_sondit_radar_data(TRadarData* ptData,ros::Publisher radar_pub,int channel,struct radar_info rada_info);


void RadarDataCB(TRadarData* ptData, WORD wDataCnt);

#endif
