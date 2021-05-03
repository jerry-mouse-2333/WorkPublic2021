#include "radar_log.h"

// string      deviceIP             #设备IP
// string      devicestatus         #设备状态 0 离线 1 在线 9 故障
// string      devicetype           #设备类型1 边端 2 摄像头 3 激光雷达（预留） 4 毫米波雷达 5 RSU
// string      logtype              #两种：“oprLog“、“syslog”
// string      loglevel             #日志级别，五种：“CRIT”、“ERR”、“WARNING”、“INFO”、“DEBUG”
// string      message              #日志内容

bool LogWriter::TimeCheck(){
    current_t = time(NULL);
    if(current_t - start_t >= 10){
        start_t = current_t;    
        return true;
    }else{
        return false;
    }
};

LogWriter::LogWriter(){
//    ROS_INFO("Radar Log Writer begin working……");
};

bool LogWriter::Init(){
    start_t = time(NULL);
    radar_log.devicetype = DeviceType;
    radar_log.logtype = LogType; 
    pub_topic_name = "log_message";
    sendlogcycle = 10;
    pub=nh.advertise<log_msgs::log>(pub_topic_name, 100);
    return true;
};

int LogWriter::UpdateDeviceNum(){
    return sock_ip_maps.size();
};


void LogWriter::ManageDevice(map <int ,vector<string>> ::iterator device){
    string ID = device->second[0];
    radar_log.deviceIP = IP_ID_map.find(ID)->second;
    radar_log.devicestatus = device->second[1];
    if (strcmp(radar_log.devicestatus.c_str(),Offline)==0)               //设备断线或故障
    {
        radar_log.loglevel = LogLevel_CRIT;
        radar_log.message = radar_log.deviceIP +" "+"Offline";
    }else if(strcmp(radar_log.devicestatus.c_str(),Malfunction)==0){     //设备发送空包（报文问题）
        radar_log.loglevel = LogLevel_ERR;
        radar_log.message = radar_log.deviceIP  +" "+ "object data in this frame is empty!";
    }else{
        radar_log.loglevel = LogLevel_DEB;
        radar_log.message = radar_log.deviceIP +" "+ "Running";
    }
};

void LogWriter::UpdateLog(){
        device_num = UpdateDeviceNum();
        std::map < int ,vector<string >> ::iterator it;        
        if (device_num != 0){
            for (it = sock_ip_maps.begin(); it != sock_ip_maps.end(); it++){       
                ManageDevice(it);
                ROS_INFO("sock_ip_maps has %d values \n",sock_ip_maps.size()); 
                pub.publish(radar_log);
            }
        }else{
            radar_log.deviceIP = " ";
            radar_log.devicestatus = "0";
            radar_log.loglevel = LogLevel_CRIT;                             //设备全部不在线
            radar_log.message = "can not get any device info !";
            pub.publish(radar_log);
        }
};

void LogWriter::run(){
    if(TimeCheck()&&ros::ok()){  
        UpdateLog();
    }
};

LogWriter:: ~LogWriter(){};


