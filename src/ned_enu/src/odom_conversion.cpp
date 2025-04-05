#include <rclcpp/rclcpp.hpp>
#include <frame_transforms.h>
// #include <node.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <chrono>
#include <memory>

#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherSubscriber : public rclcpp::Node
{
public:
  PublisherSubscriber() : Node("PublisherSubscriber")//, count_(0)
  {

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // rclcpp::Node:: for create_publisher and subcriber
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    //timer_ = this->create_wall_timer(500ms, std::bind(&PublisherSubscriber::timerCallback, this));
    subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&PublisherSubscriber::subscribeCallback, this, _1)); //_1
  }

private:

  void subscribeCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const
  {
    nav_msgs::msg::Odometry publish_data;

//this segment is nonsensical, please ignore it
    publish_data.header.stamp.sec = msg->timestamp;
    publish_data.header.stamp.nanosec = 0; //this is temporar

    Eigen::Quaterniond q = ned_enu::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
    Eigen::Quaterniond enu_q = ned_enu::frame_transforms::ned_to_enu_orientation(q);
//assumption: 0==x,1==y,2==z
    Eigen::Vector3d position = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d enu_position = ned_enu::frame_transforms::ned_to_enu_local_frame(position);
//same assumption
    Eigen::Vector3d velocity = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    Eigen::Vector3d enu_velocity = ned_enu::frame_transforms::ned_to_enu_local_frame(velocity);

    publish_data.pose.pose.position.x = enu_position.x();
    publish_data.pose.pose.position.y = enu_position.y();
    publish_data.pose.pose.position.z = enu_position.z();

    publish_data.pose.pose.orientation.x = (float)enu_q.x();
    publish_data.pose.pose.orientation.y = (float)enu_q.y();
    publish_data.pose.pose.orientation.z = (float)enu_q.z();
    publish_data.pose.pose.orientation.w = (float)enu_q.w();

    publish_data.twist.twist.linear.x = enu_velocity.x();
    publish_data.twist.twist.linear.y = enu_velocity.y();
    publish_data.twist.twist.linear.z = enu_velocity.z();

//assuming angular_velocity[0] == rollspeed, angular_velocity[1] == pitchspeed and so on
    publish_data.twist.twist.angular.x = msg->angular_velocity[0];
    publish_data.twist.twist.angular.y = -msg->angular_velocity[1];
    publish_data.twist.twist.angular.z = -msg->angular_velocity[2];
    
    publisher_->publish(publish_data);
  }

  //void timerCallback(){
    //count_++;
    //publisher_->publish(publish_data);
  //}

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;


  //rclcpp::TimerBase::SharedPtr timer_;
  //size_t count_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherSubscriber>());
  printf("cos tam");
  rclcpp::shutdown();
  return 0;
}