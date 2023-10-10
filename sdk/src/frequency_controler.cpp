#include "frequency_controler.h"

FrequencyController::FrequencyController()
{
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  LoadParams(private_nh);
  GetInitialPWM();

  laser_scan_subscriber_ = nh.subscribe("scan", 10, &FrequencyController::ScanCallback, this);

  pwm_publisher_ = nh.advertise<std_msgs::Int32>("set_pwm", 10, false);
}

void FrequencyController::GetInitialPWM()
{
  std::fstream file;
  file.open(path_to_pwm_file_, std::ios::in | std::ios::binary);
  if (file.is_open())
  {
    std::string line;
    std::getline(file, line);
    current_pwm_ = std::stoi(line);
    file.close();
  }
  else
  {
    ROS_ERROR("FAILED TO READ LIDAR PWM");
    current_pwm_ = 650;
  }
}

void FrequencyController::LoadParams(const ros::NodeHandle& private_nh)
{
  histeresis_ = private_nh.param<float>("histeresis", 0.5);
  desired_frequency_ = private_nh.param<float>("desired_frequency", 10.0);
  pwm_control_step_ = private_nh.param<int>("pwm_control_step", 30);
  number_of_msgs_to_calculate_frequency_ = private_nh.param<int>("number_of_msgs_to_calculate_frequency", 40);
  path_to_pwm_file_ = private_nh.param<std::string>("path_to_pwm_file", "/cart_configuration/cart_info/lidar_pwm.txt");
}

void FrequencyController::ControlDirection()
{
  if ((frequency_ + histeresis_) < desired_frequency_)
  {
    Control(pwm_control_step_);
  }
  else if ((frequency_ - histeresis_) > desired_frequency_)
  {
    Control(-pwm_control_step_);
  }
}

void FrequencyController::Control(int control_step)
{
  current_pwm_ += control_step;

  if (current_pwm_ >= pwm_range_.first && current_pwm_ <= pwm_range_.second)
  {
    std_msgs::Int32 msg;
    msg.data = current_pwm_;
    pwm_publisher_.publish(msg);
    SavePWMToFile(current_pwm_);
  }
  else
  {
    ROS_ERROR_STREAM("Lidar controler reached pwm limit!");
  }
}

void FrequencyController::StoreTimestamps(const double timestamp)
{
  if (last_timestamp_ != 0)
  {
    samples_.push_back(timestamp - last_timestamp_);
  }
  last_timestamp_ = timestamp;

  if (samples_.size() >= number_of_msgs_to_calculate_frequency_)
  {
    CalculateFrequency();
    samples_.clear();
  }
}

void FrequencyController::CalculateFrequency()
{
  double sum_of_samples_ = 0;
  for (const auto& v : samples_)
  {
    sum_of_samples_ += v;
  }

  frequency_ = 1.0 / (sum_of_samples_ / number_of_msgs_to_calculate_frequency_);
  ControlDirection();
}

void FrequencyController::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  StoreTimestamps(msg->header.stamp.toSec());
}

void FrequencyController::SavePWMToFile(uint pwm)
{
  std::ofstream file(path_to_pwm_file_, std::ios::out | std::ios::binary);
  if (file.is_open())
  {
    std::string data_to_write = std::to_string(current_pwm_);
    file.write(data_to_write.c_str(), data_to_write.size());
    file.close();
  }
  else
  {
    ROS_ERROR("Failed to write to pwm_file");
  }
}