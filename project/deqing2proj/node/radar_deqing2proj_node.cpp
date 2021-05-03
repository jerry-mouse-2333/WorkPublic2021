#include<typeinfo>
#include <string>
#include <vector>
#include <math.h>
#include "convert.h"


void RadarDataCB(TRadarData* ptData,WORD wDataUnit)
{
    int cb_cnt = 1;
    for(int i=0; i< wDataUnit;++i){ 
        #if 1
        {
        // printf("*************************************************************\r\n");
        printf("%d-radar id   : %s\r\n",                    cb_cnt, ptData[i].strRadarId.c_str());
        // printf("%d-frame type : %d\r\n",                    cb_cnt, ptData[i].bDataType);
        // printf("%d-frame time : %d-%d-%d-%d-%d-%d:%d \r\n", cb_cnt, ptData[i].tDataTime.wYear, ptData[i].tDataTime.bMonth, ptData[i].tDataTime.bDay, ptData[i].tDataTime.bHour, ptData[i].tDataTime.bMinute, ptData[i].tDataTime.bSecond, ptData[i].tDataTime.wMilliSec);
        // printf("%d-data count : %d\r\n",                    cb_cnt, ptData[i].dwDataLen);
        // printf("%d-datas addr : 0x%08X\r\n",                cb_cnt, ptData[i].pbData);
        }
        #endif
        if (ptData[i].pbData != nullptr){          
            std::string device_id = ptData[i].strRadarId;
            //20210410 WJJ modify sensor_source connect with roadID
            std::string radar_vali = device_id.substr(device_id.find_first_of('\0')-5,device_id.find_first_of('\0'));                               //valid ip info in radar data frame
            int index = direction_roadID_map.find(radar_vali.substr(0,2))->second-1;
            string ip = radar_vali.substr(2,5);
            switch (ptData[i].bDataType){
	            case REAL_TARGET_DATA:{
                        for (int i = 0;i<radar_num;i++){   
                            std::string config_vali = radar_info[i].radar_ip.substr(radar_info[i].radar_ip.length()-3, radar_info[i].radar_ip.length());   //valid ip info in config file                      
                            #if 1       //debug info
                            {
                            std::cout<<"i="<<i<<","<<"radar_vali: "<<radar_vali<<std::endl;
                            std::cout<<"radar_direction: "<<radar_vali.substr(0,2)<<",current config info:"<<config_vali.c_str()<<std::endl;
                            // std::cout<<strcmp(radar_vali.c_str(), config_vali.c_str()) <<std::endl;
                            std::cout<<"publish to: radar_00"<<index+1<<std::endl;
                            }
                            #endif 
                            if (strcmp(ip.c_str(), config_vali.c_str()) == 0){
                                get_sondit_radar_data(ptData, pub[index],index,radar_info[i]);
                                break;
                            }else{
                                printf(" \n");
                                continue;
                            }
                        }
                    }break;
                #if 0
                {
                    case POINT_CLOUD_DATA:
                    {
                        TPointCloudData* ptObjTmp = (TPointCloudData*)(ptData[i].pbData);
                        for (int j = 0; j < ptData[i].dwDataLen; j++)
                        {
                            printf("%d-debDistance   : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblDistance);
                            printf("%d-dblSpeed      : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblSpeed);
                            printf("%d-dblAngle      : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblAngle);
                        }
                    }
                    break;
                    case TRAFFIC_FLOW_DATA:
                    {
                        TTrafficFlowData* ptObjTmp = (TTrafficFlowData*)(ptData[i].pbData);
                        for (int j = 0; j < ptData[i].dwDataLen; j++)
                        {
                            printf("%d-wStatPeriod        : %d\r\n",    cb_cnt, ptObjTmp[j].wStatPeriod);
                            printf("%d-wLaneId            : %d\r\n",    cb_cnt, ptObjTmp[j].wLaneId);
                            printf("%d-dblMonitorPos      : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblMonitorPos);
                            printf("%d-SmallTrafficFlow   : %d\r\n",    cb_cnt, ptObjTmp[j].SmallTrafficFlow);
                            printf("%d-GrandeTrafficFlow  : %d\r\n",    cb_cnt, ptObjTmp[j].GrandeTrafficFlow);
                            printf("%d-VentiTrafficFlow   : %d\r\n",    cb_cnt, ptObjTmp[j].VentiTrafficFlow);
                            printf("%d-TotalTrafficFlow   : %d\r\n",    cb_cnt, ptObjTmp[j].TotalTrafficFlow);
                            printf("%d-dblAverageSpeed    : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblAverageSpeed);
                            printf("%d-dblTimeHeadway     : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblTimeHeadway);
                            printf("%d-dblTimeOccupancy   : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblTimeOccupancy);
                            printf("%d-dblMaxQueueLength  : %.8lf\r\n", cb_cnt, ptObjTmp[j].dblMaxQueueLength);
                        }
                    }
                    break;
                #endif
                default:
	                // printf(" other type data\r\n");
	                break;
                }
            }
    }
    cb_cnt++;
}

int main(int argc, char **argv)
{  
    if(init(argc,argv)){   
                                                                           //Write Radar Log
        signal(SIGINT, sig_handler);

        LogWriter radarlogwriter;
        radarlogwriter.Init();
        RadarProtoInit(RadarDataCB,port);                                 //recieve all data from port 
        while(1){
            radarlogwriter.run();
            RadarProtoRun(radarlogwriter);
            usleep(50*1000);	                                           //sleep 50ms   1s=1000ms
            if (app_stopped){
			break;
		}
        }
    }
    return 0;
}
