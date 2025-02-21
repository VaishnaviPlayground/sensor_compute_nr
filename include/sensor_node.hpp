/**
 * @file sensor_node.hpp
 * @brief Defines the SensorNode class for publishing sine wave sensor data in ROS2.
 */

#ifndef SENSOR_NODE_HPP
#define SENSOR_NODE_HPP

//Ros2 dependencies header
#include "rclcpp/rclcpp.hpp"

//Custom message header to pubish stamped sine waves
#include "sensor_compute/msg/sensor_timestamp.hpp"

using namespace std;
/**
 * @class SensorNode
 * @brief A ROS2 node that simulates a sine wave sensor.
 * 
 * The SensorNode generates timestamped sine wave data at 500Hz and
 * publishes it on a ROS2 topic.
 */
class SensorNode : public rclcpp::Node
{
    public:

    /**
     * @brief Constructor to create a SensorNode object.
     * @param sensor_name The name of the sensor, used as the topic name.
     * 
     * an explicit constructor initializes the ROS2 node, the publisher, and 
     * initializes a timer to publish sine wave data periodically.
     */
    explicit SensorNode(const std::string  sensor_name);

    private:

    /**
     * @brief Publishes sine wave data with a timestamp.
     * 
     * This function generates a sine wave value, attaches a timestamp, and 
     * publishes the message to the corresponding ROS2 topic.
     */
    void publish_sinewave();

    rclcpp::Publisher<sensor_compute::msg::SensorTimestamp>::SharedPtr publisher_;  ///<Publisher for sensor data with time stamps
    rclcpp::TimerBase::SharedPtr timer_;  ///<Timer to periodically publish data

};

#endif  //SENSOR_NODE_HPP