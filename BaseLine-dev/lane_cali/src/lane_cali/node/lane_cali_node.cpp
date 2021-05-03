
#include "lane_cali.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_cali");
    LaneCali App;
    App.Run();
    return 0;
}
