
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "navx_lib/AHRS.h"
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{

private:

  AHRS imu = AHRS("/dev/ttyACM0"); 
  const float GRAVITY_ACCEL = 9.80665f; // m/s^2 - The IMU lib uses this specific value of g.
  const float PI = 3.14159f;
  const float TO_RADS = PI / 180;
  const std::string FRAME_ID = "imu_link"; // Name of the ROS tf frame ID

  const std::array<double, 9> linear_accel_covariance_mat  = {
    0.01, 0, 0,
    0, 0.01, 0,
    0, 0, 0.01
  };


public:

  MinimalPublisher() : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("demo/imu", 10);
    
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MinimalPublisher::timer_callback, this));

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

private:

  void timer_callback()
  {
    auto message = sensor_msgs::msg::Imu();
    
    // Set msg header
    message.header.frame_id = FRAME_ID;
    message.header.stamp = this->now();

    // Set orientation
    message.orientation.x = imu.GetQuaternionX();
    message.orientation.y = imu.GetQuaternionY();
    message.orientation.z = imu.GetQuaternionZ();
    message.orientation.w = imu.GetQuaternionW();

    // Set angular velocity (In rad/sec)
    // message.angular_velocity.x = imu.GetRawGyroX() * TO_RADS;
    // message.angular_velocity.y = imu.GetRawGyroY() * TO_RADS;
    // message.angular_velocity.z = imu.GetRawGyroZ() * TO_RADS;

    // Set linear acceleration (In m/s^2)
    message.linear_acceleration.x = 0;//imu.GetWorldLinearAccelX() * 100 / GRAVITY_ACCEL;
    message.linear_acceleration.y = 0;//imu.GetWorldLinearAccelY() * 100 / GRAVITY_ACCEL;
    message.linear_acceleration.z = 0;//imu.GetWorldLinearAccelZ() / GRAVITY_ACCEL;

    // message.linear_acceleration_covariance = linear_accel_covariance_mat;

    // message.linear_acceleration.x = imu.GetRawAccelX() / GRAVITY_ACCEL;
    // message.linear_acceleration.y = imu.GetRawAccelY() / GRAVITY_ACCEL;
    // message.linear_acceleration.z = imu.GetRawAccelZ() / GRAVITY_ACCEL;
    
    // std::cout << "X: " << imu.GetDisplacementX() << std::endl;
    // std::cout << "Y: " << imu.GetDisplacementY() << std::endl;
    // std::cout << "Z: " << imu.GetDisplacementZ() << std::endl;

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  rclcpp::shutdown();

  return 0;
}
