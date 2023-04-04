#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/use", 10);
      gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/navsatfix/use", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
      gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/navsatfix", 10, std::bind(&MinimalPublisher::gps_callback, this, _1));
    }

  private:

    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        auto message = *msg;
        std::array<double, 9> covar = msg->orientation_covariance;
        std::array<double, 9> covar_acc = msg->orientation_covariance;
        std::array<double, 9> covar_angv = msg->orientation_covariance;
        covar[0] = 2*1e-5;
        covar[4] = 2*1e-5;
        covar[8] = 2*1e-5;
        covar_acc[0] = 1;
        covar_acc[4] = 1;
        covar_acc[8] = 1;
        covar_angv[0] = 1;
        covar_angv[4] = 1;
        covar_angv[8] = 1;
        message.orientation_covariance = covar;
        message.angular_velocity_covariance = covar_angv;
        message.linear_acceleration_covariance = covar_acc;
    //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        publisher_->publish(message);
    }
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        auto message = *msg;
        std::array<double, 9> gps_covar = msg->position_covariance;
        double lat = msg->latitude;
        double lon = msg->longitude;
        if(abs(gps_covar[0])>1000 || abs(lat) > 180 || abs(lon) >180)
        {
            lat = lat_old_;
            lon = lon_old_;
            std::array<double, 9> gps_covar_new;
            gps_covar_new[0]= 10000;
            gps_covar_new[4]= 10000;
            gps_covar_new[8]= 10000;
            gps_covar = gps_covar_new;
            // std::cout << "JUMP at time" << rclcpp::Clock{RCL_ROS_TIME}.now().seconds()<< std::endl;
            // std::cout << "lat" << " " << lat << std::endl;
            // std::cout << "lon" << " " << lon << std::endl;
            // std::cout << "gps_covar[0]" << " " << gps_covar[0] << std::endl;
        }
        else
        {
            lat_old_ = lat;
            lon_old_ = lon;
        }
        message.latitude = lat;
        message.longitude = lon;
        message.position_covariance = gps_covar;
        gps_publisher_->publish(message);
    }

    void timer_callback()
    {
    //   auto message = sensor_msgs::msg::Imu();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    double lat_old_, lon_old_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}