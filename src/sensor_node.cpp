#include "sensor_node.hpp"
//Math header for calculating sine wave
#include <cmath> 
//reusability
using sensor_type = sensor_compute::msg::SensorTimestamp;

SensorNode::SensorNode(std::string sensor_name) : Node(sensor_name) 
{   
    //publisher to publish sin wave and publish topic like _
    publisher_ = this->create_publisher<sensor_type>(sensor_name + "_topic", 10);
    //timer every 500hz 1/f =2ms
    timer_ = this->create_wall_timer(2ms, std::bind(&SensorNode::publish_sinewave, this));
}

void SensorNode::publish_sinewave() 
{
    auto message = sensor_type();
    //always using rclcpp::Time so synchronization is done
    //TODO: what happens if both nodes run of different computers? ptp linux
    message.header.stamp = this->now();
    // 2*pi*f*t calculates wave phase assuming default f=1 one complete oscilation and time is the current time
    message.data = std::sin(2.0 * M_PI * 1.0 * rclcpp::Time(message.header.stamp).seconds());
    // publish
    publisher_->publish(message);
}



int main(int argc, char **argv) 
{
    //init
    rclcpp::init(argc, argv);
    //two sensors created
    auto sensor1 = std::make_shared<SensorNode>("sensor1");
    auto sensor2 = std::make_shared<SensorNode>("sensor2");
    // Multi threading since i have two instances of the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensor1);
    executor.add_node(sensor2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
