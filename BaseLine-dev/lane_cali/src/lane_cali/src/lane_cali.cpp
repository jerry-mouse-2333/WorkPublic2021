#include "lane_cali.h"
bool    app_stopped = false;
void LaneCali::LidarTransform(vector<float> trans_,struct Point * p){   
    Eigen::Translation3f init_translation(trans_[0],
                                          trans_[1],
                                          trans_[2]);
    Eigen::AngleAxisf     init_rotation_x(trans_[3], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf     init_rotation_y(trans_[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf     init_rotation_z(trans_[5], Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation_matrix_ ;
    rotation_matrix_ = init_rotation_x * init_rotation_y  * init_rotation_z;

    // Eigen::Matrix4f trans_matrix_;
    // trans_matrix_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    Eigen::Vector3f v_p(p->x,p->y,p->z);
    v_p = rotation_matrix_.matrix()*v_p+init_translation.vector();
    // cerr<<"rotation: "<<rotation_matrix_<<endl;
        // cout<<"before---x: "<<p->x<<"  y: "<<p->y<<endl;
    p->x = v_p[0];
    p->y = v_p[1];
    p->z = v_p[2];

    // cout<<"after---x: "<<p->x<<"  y: "<<p->y<<endl;
}
void LaneCali::LoadLanePos(string configPath_str,map<string,struct iLaneInfo>* lanes){
	ifstream ifs;
	ifs.open(configPath_str.c_str());
	if (!ifs.is_open()) {
        cout<<"[LanePos Reading]: cannot open config !"<<endl;
		return ;
	}
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (!root["center"].isNull()) {
			int nSize = root["center"].size();
            bool deal = false;    
			for (int i = 0;i < nSize; ++i){
                pair<string,struct iLaneInfo> lane;
                lane.first = root["center"][i]["Direction"].asString();
                ofstream ofs;  
                ofs.open(("/home/jingjing/lane_cali/src/lane_cali/result/"+lane.first+".txt").c_str(),ios::trunc);
                lane.second.mainLidar_= root["center"][i]["Main_lidar"].asString();
                lane.second.angle = root["center"][i]["Angle"].asInt();
                lane.second.stopLine_ = root["center"][i]["StopLine"].asInt();  
                lane.second.laneAxis_ = root["center"][i]["LaneAxis"].asString();                
                if (lane.second.mainLidar_!= "yes"){
                    deal = true;
                }
                int stop_line = 0; //以来向的均值为stop_line参考，暂不考虑去向
			    for (int j = 0; j < root["center"][i]["Come_close_cross_center"].size(); j++) {                   
                    struct Point p;
                    float x = root["center"][i]["Come_close_cross_center"][j]["x"].asFloat();
                    float y = root["center"][i]["Come_close_cross_center"][j]["y"].asFloat();
                    float pos_x = (x)*cos(0.01745*(lane.second.angle))+(y)*sin(0.01745*(lane.second.angle));
                    float pos_y = (y)*cos(0.01745*(lane.second.angle))-(x)*sin(0.01745*(lane.second.angle)); 
                    p.x = pos_x;
                    p.y = pos_y;
                    p.z = -3.5;
                    if (deal){
                        LidarTransform(trans_,&p);
                    }  
                    if (strcmp(lane.second.laneAxis_.c_str(), "x")==0)
                    {
                        stop_line += p.y;
                    }else if(strcmp(lane.second.laneAxis_.c_str(), "y")==0){
                        stop_line += p.x;
                    }                
                    ofs  <<"come_point_"<<j<<"  "<< "x = " <<p.x<<"  y=  "<<p.y<<endl;
                    lane.second.come_array_.push_back(p);
                }
                lane.second.stopLine_ = stop_line/lane.second.come_array_.size();
                for (int j = 0; j < root["center"][i]["Leave_away_cross_center"].size(); j++) {
                    struct Point p;
                    float x = root["center"][i]["Leave_away_cross_center"][j]["x"].asFloat();
                    float y = root["center"][i]["Leave_away_cross_center"][j]["y"].asFloat();
                    float  pos_x = (x)*cos(0.01745*(lane.second.angle))+(y)*sin(0.01745*(lane.second.angle));
                    float  pos_y = (y)*cos(0.01745*(lane.second.angle))-(x)*sin(0.01745*(lane.second.angle)); 
                    p.x = pos_x;
                    p.y = pos_y;
                    p.z = -3.5;
                    if (deal){
                        LidarTransform(trans_,&p);
                    }  
                    if (strcmp(lane.second.laneAxis_.c_str(), "x")==0)
                    {
                        stop_line += p.y;
                    }else if(strcmp(lane.second.laneAxis_.c_str(), "y")==0){
                        stop_line += p.x;
                    }  
                    ofs  <<"leave_point_"<<j<<"  "<< "x = " <<p.x<<"  y=  "<<p.y<<endl;
                    lane.second.leave_array_.push_back(p);
			    } 
            ofs.close();    
            lanes->insert(lane);
		}
     
	}
	ifs.close();

    }
	return;   
}

bool LaneCali::Init(){

    // TODO:
    W_stop_line = -27;
    S_stop_line = 61;

    E_stop_line = 35;
    N_stop_line = 9.5;
    ros::NodeHandle priv_nh("~");
    string configPath_str;
    string tranMatrix_str;
    vector<string> tranMatrix_;
    priv_nh.getParam("/config_dir",configPath_str); 
    priv_nh.getParam("/sub_topic",sub_topic_);
    priv_nh.getParam("/pub_topic",pub_topic_); 
    priv_nh.getParam("/tranMatrix",tranMatrix_str);
    priv_nh.getParam("/dist_before_stop_line",l1);
    priv_nh.getParam("/dist_after_stop_line",l2);
    priv_nh.getParam("/cordinate_rotation",angle);
    // cout<<"l1: "<<l1<<" l2: "<<l2<<"  angle: "<<angle<<endl;
    tranMatrix_ = split_function(tranMatrix_str,",");
    for(int i = 0 ; i < tranMatrix_.size() ; ++i){
        trans_.push_back(atof(tranMatrix_[i].c_str()));
    }
    LoadLanePos(configPath_str,&lanes); 
    return true;
}

uint8_t LaneCali::assign(struct iLaneInfo lane,float pos_x,float pos_y){
    uint8_t lane_id = 255;
    size_t c_t = lane.come_array_.size();    
    
    //x坐标vector
    vector<float> v_x(c_t);
    for (size_t i = 0; i< c_t ; i++){ 
                v_x[i] = lane.come_array_[i].x;
    }
    sort(v_x.begin(),v_x.end()); 
    int width_x = static_cast<int>((v_x.back() - v_x[0])/(c_t-1)) ;

    //y坐标vector
    vector<float> v_y(c_t);
    for (size_t i = 0; i< c_t ; i++){ 
                v_y[i] = lane.come_array_[i].y;
    }
    sort(v_y.begin(),v_y.end());
    int width_y =  static_cast<int>((v_y.back() - v_y[0])/(c_t-1));

    if (strcmp(lane.laneAxis_.c_str(), "x")==0){
        for (uint8_t i = 1; i< c_t ; i++){   
            cout<<"v_x: "<<  v_x[i-1]<<endl;;                 
            if( pos_x>v_x[0] && pos_x<=v_x[0]+width_x*i) {
                lane_id =10+i; 
                cout<<"   lane_id: "<<unsigned(lane_id)<<endl;  
                break;                                  
            }else{
                continue;                                                                    
            }
        }        
    }else if (strcmp(lane.laneAxis_.c_str(), "y")==0){       
        for (uint8_t i = 1; i< c_t ; i++){  
            cout<<"v_y"<<  v_y[i-1]<<endl;                     
            if( pos_y>v_y[0] && pos_y<=v_y[0]+width_y*i) {
                lane_id =10+i;  
                cout<<"   lane_id: "<<unsigned(lane_id)<<endl; 
                break;                                  
            }else{
                continue;                                                                    
            }
        }       
    }
    return lane_id;
}

int LaneCali::AssignLaneIdAndSensorId(geometry_msgs::Pose obj_pose){
    // cout<<" pos_x:  "<<obj_pose.position.x<<" ,pos_y:  "<<obj_pose.position.y<<endl;//,int* roadID
    float pos_x = obj_pose.position.x;
    float pos_y = obj_pose.position.y;
    
    //确定object方位=N/S/W/E/
    uint8_t lane_id = 0;

    if (S_stop_line-l2<=pos_x && pos_x<=S_stop_line+l1){
            map<string,struct iLaneInfo>::iterator l;  
            l = lanes.find("South");
            if (l!= lanes.end()){
                cout<<"South:   "<<l->second.mainLidar_<<endl;
                // &roadID = 1;

                lane_id = assign(l->second,pos_x,pos_y);
            }
    }else  if( N_stop_line-l2<=pos_x && pos_x<=N_stop_line+l1){
           //direction=="North", x方向为停止线
            map<string,struct iLaneInfo>::iterator l;  
            l = lanes.find("North");

            if (l!= lanes.end()){
                // &roadID = 1;
                for (size_t i = 0; i< l->second.come_array_.size(); i++){ 
                            l->second.come_array_[i].y = -1*(l->second.come_array_[i].y+2);
                }
                cout<<"North:   "<<l->second.mainLidar_<<endl;
                lane_id = assign(l->second,pos_x,-pos_y);
            }  
    
    }else if( W_stop_line-l1<=pos_y && pos_y<=W_stop_line+l2){

            map<string,struct iLaneInfo>::iterator l;  
            l = lanes.find("West");
            cout<<"West:   "<<l->second.mainLidar_<<endl;

            if (l!= lanes.end()){
                // &roadID = 1;
                for (size_t i = 0; i< l->second.come_array_.size(); i++){ 
                            l->second.come_array_[i].x = -1*(l->second.come_array_[i].x+1.5);
                }
                lane_id = assign(l->second,-pos_x,pos_y);
            }
    }
    else if( E_stop_line-l2<=pos_y && pos_y<=E_stop_line+l1) {
            map<string,struct iLaneInfo>::iterator l;  
            l = lanes.find("East");
            cout<<"East:   "<<l->second.mainLidar_<<endl;

            if (l!= lanes.end()){

                // &roadID = 1;
                for (size_t i = 0; i< l->second.come_array_.size(); i++){ 
                            l->second.come_array_[i].x = -1*(l->second.come_array_[i].x-0.8);
                }
                lane_id = assign(l->second,-pos_x,pos_y);
            //         size_t c_t = l->second.come_array_.size(); 
            //         for (int i = 0; i< c_t ; i++){                          
            //             if( pos_y>l->second.come_array_[i].x && pos_y<=l->second.come_array_[i+1].x ) {
            //                 lane_id =9+l->second.come_array_.size()-i;                                       
            //             }else{
            //                 continue;                                                                    // objects out of lane range will not be dealed
            //             }
            //         }
            //         if (lane_id == 255){
            //             for (int i = 0; i< l->second.leave_array_.size(); i++){                          
            //                 if( pos_y>l->second.leave_array_[i].x && pos_y<=l->second.leave_array_[i+1].x ) {
            //                     lane_id =31+i;                                    
            //                 }else{
            //                     continue;                                                                    // objects out of lane range will not be dealed
            //                 }
            //             }
            //         }
            }
    }  
    // cout<<*roadID<<endl;  
    return lane_id;
}

void LaneCali::CallBack(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
{  
    autoware_msgs::DetectedObjectArray A_array; 
    A_array.header = msg->header;
    A_array.sensorname = msg->sensorname;
    A_array.objects = msg->objects;

    size_t n_size = msg->objects.size();
    if (n_size!=0){
        for (int i = 0 ; i < n_size ; ++i){

            int roadID = 0;

            A_array.objects[i].lane_id = AssignLaneIdAndSensorId(A_array.objects[i].pose);//,&roadID); 
            // cout<<"lane_id in object: "<<unsigned(A_array.objects[i].lane_id)<<endl;
            A_array.objects[i].sensor_source = roadID;
        };
        pub_.publish(A_array);
    }
}
void sig_handler( int sig )
{
    if ( sig == SIGINT){
        app_stopped = true;
    }
}

void LaneCali::Run(){
    if (this->Init()){
        sub_ = nh.subscribe(sub_topic_,10,&LaneCali::CallBack,this);
        pub_ = nh.advertise<autoware_msgs::DetectedObjectArray>(pub_topic_, 10);
        signal(SIGINT, sig_handler);
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
            if (app_stopped){
                break;
            }
        }
    }

}