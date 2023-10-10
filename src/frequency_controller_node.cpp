#include <ros/ros.h>


#include "frequency_controler.h" 



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lidar_frequency_controller");
    ros::NodeHandle nh_;
    auto controller = FrequencyController();
    
    ros::spin();

    return 0;
}