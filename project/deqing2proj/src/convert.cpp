#include "convert.h"
#include<sys/types.h>
#include<dirent.h>
#include "json/json.h" 
#include <cmath>
#include <fstream>
#define cal 1
const float PI = 3.1415926f;

void sig_handler( int sig )
{
    if ( sig == SIGINT){
        app_stopped = true;
    }
}


float* split_function(string input_str,const char *split_signal,int elements_num){    
    float *result_array = new float[elements_num];
    char *element_p = NULL;
    char* c = new char[input_str.length()+1];
    strcpy(c,input_str.c_str());
    element_p = strtok(c, split_signal);
    result_array[0] = atof(element_p);
    for (int i = 1;i<elements_num;i++){
        if(element_p!=NULL){
            element_p = strtok(NULL, split_signal);    
        }
    result_array[i] = atof(element_p);                
    } 
    return result_array;
}

//20210326 modify config name and reading method  
vector<struct radar_info> loadRadarInfo(string param_dir){
    std::vector<struct radar_info> radar_info_;
    std::string close_lane,away_lane;
    Json::Value root;
	Json::Reader reader;    
    DIR *dir = opendir(param_dir.c_str()); 
    struct dirent *entry;  
    string name = "default";        
    if (dir == NULL)        {
        cout << "opendir error" << endl;
    }else{
        while ((entry = readdir(dir)) != NULL)
        {              
            name = entry->d_name;           
            if (entry->d_type == DT_REG){ 
                std::ifstream inputjson((param_dir+name).c_str());        
                if (!inputjson.is_open()){
                    std::cout << "cannot open config file !"<< std::endl;
                    std::cout <<"make sure your config file exist in: "<<(param_dir+name).c_str() << std::endl;
                    exit(0);
                }else{
                    struct radar_info radar; 
                    if (reader.parse(inputjson, root)) {
                        radar.radar_ip = root["radar_ip"].asString();  
                        // radar.device_id = root["device_id"].asString();
                        radar.vehicle_stop_line = root["vehicle_stop_line"].asInt();                         
                        radar.close_direction_lane_num = root["close_direction_lane_num"].asInt(); 
                        close_lane= root["close_lane"].asString();
                        radar.close_direction_lane_num = root["away_direction_lane_num"].asInt(); 
                        away_lane= root["away_lane"].asString();
                        radar.close_lane = new float[radar.close_direction_lane_num];
                        radar.away_lane = new float[radar.away_direction_lane_num];
                    }
                    inputjson.close();
                    radar.close_lane = split_function(close_lane,",",radar.close_direction_lane_num+1);
                    radar.away_lane = split_function(away_lane,",",radar.close_direction_lane_num+1);
                    radar_info_.push_back(radar);
                }}
        }               
    }      
    return radar_info_;
}


bool init(int argc, char **argv){
    ros::init(argc, argv, "radar_convert");
    ros::NodeHandle nh;    
    ros::NodeHandle pri_nh("~");
    pri_nh.getParam("radar_num", radar_num);
    pri_nh.getParam("port", port);
    pri_nh.getParam("radar_information_dir", param_dir);
    pri_nh.getParam("same_direction", same_direction);
    #if 0                                                                          //debug info 
    {            
    ROS_INFO("radar config file dir : %s\n",(param_dir+"/config/").c_str());
    ROS_INFO("radar_num : %d\n",radar_num);
    ROS_INFO("same_direction : %d\n",same_direction);
    }
    #endif  
    pub.resize(radar_num);                                                          //resize Publishers and read radar_info to vector
    radar_info = loadRadarInfo(param_dir+"/config/"); 
    int l = 0;
    for (int i = 0;i < radar_num;i++){
            pub[i] = nh.advertise<radar_msgs::RadarObjectsArray>(adv_name+"00"+std::to_string(i+1), 1000);
    }
    return true;
}

//2021-01-28 WJJ add velocity field
// 2021-03-18 WJJ add explanation: unit:km/h     //modify lane_id: only "close" direction
static void obj_data_convert(const TObjectData* ptObjTmp,
                            radar_msgs::RadarObjects_General & radar_obj_general,int i,radar_msgs::RadarObjectsArray* radar_obj_array,int channel,
                            struct radar_info radar_info)
{
    radar_obj_general.header.seq        = i+1;
    radar_obj_general.header.stamp.sec  = (radar_obj_array->header).stamp.sec;       
    radar_obj_general.header.stamp.nsec = (radar_obj_array->header).stamp.nsec;
    radar_obj_general.header.frame_id   = (radar_obj_array->header).frame_id;
    radar_obj_general.sensor_source = channel;
    radar_obj_general.id                = static_cast<uint32_t>(ptObjTmp->wTargetId);
    radar_obj_general.pos_x             = static_cast<float>(ptObjTmp->dblXCoordinates);
    radar_obj_general.pos_y             = static_cast<float>(ptObjTmp->dblYCoordinates);
    radar_obj_general.Vx                = static_cast<float>(ptObjTmp->dblXaxisVelocity);
    radar_obj_general.Vy                = static_cast<float>(ptObjTmp->dblYaxisVelocity);
    radar_obj_general.lat               = static_cast<float>(ptObjTmp->dblTargetLatitude);
    radar_obj_general.lon               = static_cast<float>(ptObjTmp->dblTargetLongitude);
    radar_obj_general.DynProp           = static_cast<float>(ptObjTmp->dblVelocity);
    uint8_t lane_id                     = static_cast<uint8_t>(ptObjTmp->bLaneId);
    uint8_t close_lane_num = radar_info.close_direction_lane_num;
    if(radar_obj_general.Vy<=0&&radar_obj_general.pos_y<radar_info.vehicle_stop_line){      //e.g.: radar_pos_y:0,stop_line:75,only pos_y<75will be dealed
        for (i = 0; i< close_lane_num; i++){                                                // re-assign and keep lane_id: e.g.array = "-1.6,-4.5,-7.5,-10.6,-13.6" corresponding lane_id: 11 12 13 14           
            if((radar_obj_general.pos_x<=radar_info.close_lane[i])&&(radar_obj_general.pos_x>radar_info.close_lane[i+1])) {
                #if 1
                {
                printf("sensor_source: %d,pos_x: %f,pos_y: %f,Vy: %f,lane_id: %d, modified to: %d\n",radar_obj_general.sensor_source,radar_obj_general.pos_x,radar_obj_general.pos_y,radar_obj_general.Vy,lane_id,11+i);  
                }
                #endif
                lane_id =11+i;                                      
            }else{
                continue;                                                                    // objects out of lane range will not be dealed
            }
        }
    }
    radar_obj_general.lane_id          = lane_id;    
    radar_obj_general.type             = static_cast<uint8_t>(ptObjTmp->bTargetType);        //0:small car 1：large car 2：X-large car 3：bike
    //转为弧度
    if(ptObjTmp->dblTargetCourseAngle > 180)
        radar_obj_general.OrientationAngle     = static_cast<float>((ptObjTmp->dblTargetCourseAngle - 360) * PI / 180 );
    else
        radar_obj_general.OrientationAngle     = static_cast<float>((ptObjTmp->dblTargetCourseAngle) * PI / 180 );   
    radar_obj_general.length           = static_cast<float>(ptObjTmp->dblTargetLength);
    radar_obj_general.width            = static_cast<float>(ptObjTmp->dblTargetWidth);

    #if 0                                                                                    //debug info
    {
    printf("sec =  %d  nsec = %d\n id = %d  label =  %d  lane_id =  %d  sensor_source = %d\n radar_px = %f  radar_py = %f  OrientationAngle = %f\n radar_vx = %f  radar_vy = %f  velocity = %f\n lat = %f  lon = %f  length = %f  width =  %f\n",
                radar_obj_general.header.stamp.sec,radar_obj_general.header.stamp.nsec,
                radar_obj_general.id,radar_obj_general.type,radar_obj_general.lane_id,channel,
                radar_obj_general.pos_x,radar_obj_general.pos_y,radar_obj_general.OrientationAngle,
                radar_obj_general.Vx,radar_obj_general.Vy,radar_obj_general.DynProp,
                radar_obj_general.lat,radar_obj_general.lon,radar_obj_general.length,radar_obj_general.width );
    }
    #endif
}

static void dump_rt_data(TRadarData* ptData,ros::Publisher radar_pub,int channel,struct radar_info radar_info)//传入ptData[i]
{
    int i = 0;   
    uint32_t struct_sec =0;   
    uint32_t struct_nsec = 0; 
    radar_msgs::RadarObjectsArray     radar_obj_array;
    radar_msgs::RadarObjects_General  radar_obj_general;
    radar_msgs::RadarPosition         radar_pos;
    radar_obj_array.header.frame_id = "world"; 
    int object_num = ptData->dwDataLen;
    radar_obj_array.objects_general.resize(object_num); 
    #if 0                                                                                   //debug info 
    {
        // printf("radar Timestamp: %ld-%ld-%ld-%ld-%ld-%ld-%ld\n", ptData->tDataTime.wYear, ptData->tDataTime.bMonth, ptData->tDataTime.bDay, ptData->tDataTime.bHour, ptData->tDataTime.bMinute, ptData->tDataTime.bSecond, ptData->tDataTime.wMilliSec);
        int cb_cnt = 1;
        TObjectData* ptObjTmp = (TObjectData*)(ptData->pbData);
        for (int j = 0; j < ptData[i].dwDataLen; j++){
            printf("%d-wTargetId           : %d\r\n",	   cb_cnt, ptObjTmp[j].wTargetId);
            printf("%d-bLaneId             : %d\r\n",	   cb_cnt, ptObjTmp[j].bLaneId);
            printf("%d-bTargetType         : %d\r\n",	   cb_cnt, ptObjTmp[j].bTargetType);
            printf("%d-dblTargetLength     : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblTargetLength);
            printf("%d-dblTargetWidth      : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblTargetWidth);
            printf("%d-dblTargetHeight     : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblTargetHeight);
            printf("%d-dwReserve           : 0x%08X\r\n",  cb_cnt, ptObjTmp[j].dwReserve);
            printf("%d-dblTargetCourseAngle: %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblTargetCourseAngle);
            printf("%d-dblXCoordinates     : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblXCoordinates);
            printf("%d-dblYCoordinates     : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblYCoordinates);
            printf("%d-dblXaxisVelocity    : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblXaxisVelocity);
            printf("%d-dblYaxisVelocity    : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblYaxisVelocity);
            printf("%d-dblVelocity         : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblVelocity);
            printf("%d-dblAcceleration     : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblAcceleration);
            printf("%d-dblXaxisAcceleration: %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblXaxisAcceleration);
            printf("%d-dblYaxisAcceleration: %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblYaxisAcceleration);
            printf("%d-dblTargetLongitude  : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblTargetLongitude);
            printf("%d-dblTargetLatitude   : %.8lf\r\n",   cb_cnt, ptObjTmp[j].dblTargetLatitude);
        }
    }
    #endif
    struct timeval tv;
    bzero(&tv, sizeof(tv));
    gettimeofday(&tv,NULL);
    struct tm time;        //struct time,if neccessary, assign sec&&nsec to timestamp
    time.tm_year = ptData->tDataTime.wYear-1900;
    time.tm_mon = ptData->tDataTime.bMonth-1;
    time.tm_mday = ptData->tDataTime.bDay;
    time.tm_hour = ptData->tDataTime.bHour+8;
    time.tm_min = ptData->tDataTime.bMinute;
    time.tm_sec =ptData->tDataTime.bSecond ; 
    struct_sec  = mktime(&time);
    struct_nsec = static_cast<uint32_t>(ptData->tDataTime.wMilliSec*1000000);
    if (cal==0){
        radar_obj_array.header.stamp.sec  = struct_sec;
        radar_obj_array.header.stamp.nsec = struct_nsec;
    }else if(cal==1){
        radar_obj_array.header.stamp.sec  =ros::Time::now().sec;                          //assign ros time to each object
        radar_obj_array.header.stamp.nsec = ros::Time::now().nsec;
    }
    
    switch (object_num > 0){
        case 0:{
            ROS_INFO("=========sensor_source:[%d]===========time:[%d], =============>>>>>>>>ObjCount: %d\n",
                channel, radar_obj_array.header.stamp.sec,ptData->dwDataLen);
        }break;
        case 1:{   
            radar_obj_array.object_num =  static_cast<uint32_t>(object_num);     
            TObjectData* ptObjTmp = (TObjectData*)(ptData->pbData);
            for(i = 0; i< object_num;++i){
                memset(&radar_obj_general, 0, sizeof(radar_msgs::RadarObjects_General));
                obj_data_convert(&ptObjTmp[i],radar_obj_general,i,&radar_obj_array,channel,radar_info);
                // obj_data_convert(ptData->pbData[i],radar_obj_general,i,&radar_obj_array,channel,radar_info);
                radar_obj_array.objects_general[i] = radar_obj_general;
            }           
        }break;
        default:
            printf("object num error \r\n");
            break;
    }
    radar_pub.publish(radar_obj_array);
}


void* get_sondit_radar_data(TRadarData* ptData,ros::Publisher radar_pub,int channel,struct radar_info radar_info)
{
		dump_rt_data(ptData,radar_pub,channel,radar_info);
}



