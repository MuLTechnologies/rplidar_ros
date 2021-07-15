#ifndef FREQUENCY_CONTROLLER_H
#define FREQUENCY_CONTROLLER_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

#include <filesystem>
#include <fstream>

class FrequencyController
{
    public:
    FrequencyController();
    
    void GetInitialPWM();
    void LoadParams(const ros::NodeHandle & private_nh);

    void ControlDirection();
    void Control(int control_step);
    void CalculateFrequency();
    void StoreTimestamps(const double timestamp);

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);
    void SavePWMToFile(uint pwm);

    protected:

    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher pwm_publisher_;

    float histeresis_;
    float desired_frequency_;
    int pwm_control_step_;
    int number_of_msgs_to_calculate_frequency_;
    int current_pwm_;
    std::string path_to_pwm_file_;


    const std::pair<uint, uint> pwm_range_{200, 800};

    double last_timestamp_ = 0;
    float frequency_;
    std::vector<double> samples_;

};

#endif