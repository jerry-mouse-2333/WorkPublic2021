#ifndef  __RADAR_LOG_H__
#define  __RADAR_LOG_H__
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include "time.h"
#include "stdio.h"
#include "log_msgs/log.h"
using namespace std;

// string      deviceIP             #设备IP
// string      devicestatus         #设备状态 0 离线 1 在线 9 故障
// string      devicetype           #设备类型1 边端 2 摄像头 3 激光雷达（预留） 4 毫米波雷达 5 RSU
// string      logtype              #两种：“oprLog“、“syslog”
// string      loglevel             #日志级别，五种：“CRIT”、“ERR”、“WARNING”、“INFO”、“DEBUG”
// string      message              #日志内容


#define Online         "1"
#define Offline        "0"
#define Malfunction    "9"
#define DeviceType     "4"
#define LogType        "syslog"
#define LogLevel_CRIT   "CRIT"
#define LogLevel_ERR    "ERR"
#define LogLevel_WAR    "WARNING"
#define LogLevel_INFO   "INFO"
#define LogLevel_DEB    "DEBUG"

typedef map<int,vector<string>> DeviceInfo;

static map<string,string> IP_ID_map = {
    {"RG1717SDNE116","18.16.50.116"},
    {"RG1719SDSE117","18.16.50.117"},
    {"RG1721SDNW118","18.16.50.118"},
    {"RG1723SDSW119","18.16.50.119"},
    {"RG1781SDSW120","18.16.50.120"}
};

class LogWriter{

public:
    LogWriter();
    virtual ~LogWriter();
    friend class CMecRadarPro;
    bool Init();                                 //write fixed info of this node, and init members
    bool TimeCheck();                          //check if it's time to check&update manage list (true to execute)
//依赖于sock_ip_maps
    void ManageDevice(map <int ,vector<string>> ::iterator device);
    int UpdateDeviceNum();
    void UpdateLog();
    void run();

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    //TO DO :launch
    string pub_topic_name;
    int sendlogcycle; //unit:sec
    log_msgs::log radar_log;
    int device_num;
    time_t current_t,start_t;
    DeviceInfo sock_ip_maps;
};

#endif