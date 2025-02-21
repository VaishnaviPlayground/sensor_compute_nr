/**
 * @file central_node.hpp
 * @brief Defines the central node class for subscribing to two sensors and calculates latency and .
 */

 #ifndef CENTRAL_NODE_HPP
 #define CENTRAL_NODE_HPP

//Ros2 dependencies header
#include "rclcpp/rclcpp.hpp"

//Custom message header to pubish stamped sine waves
#include "sensor_compute/msg/sensor_timestamp.hpp"

/**
 * @class CentralNode
 * @brief A ROS2 node that subscribes to two sensor topics and calculates latency and ensures time is synchronized.
 */

 class CentralNode: public rclcpp::Node 
 {
    public:
    /**
    * @brief Constructor for CentralNode.
    */
    CentralNode();

    private:
    /**
    * @brief Callback for sensor1 data to subscribe.
    */
    void sensor1_subscribe(const sensor_compute::msg::SensorTimestamp::SharedPtr msg);

    /**
    * @brief Callback for sensor2 data to subscribe
    */
    void sensor2_subscribe(const sensor_compute::msg::SensorTimestamp::SharedPtr msg);

    /**
    * @brief save the sensor values in a log
    */
    void latency_measurement(const std::string &sensor_name, const sensor_compute::msg::SensorTimestamp::SharedPtr msg);
    
    /**
    * @brief Computes the average latency from a vector of latency values.
    * @param latencies its the sensor latencies vector containing latency values.
    * @return The computed average latency.
    */
    double compute_average_latency(const std::vector<double>& latencies);

    /**
    * @brief data used later can be used for plotting and also storing average latencies for 10 second intervals
    */
    void wave_sensor_data();



    rclcpp::Subscription<sensor_compute::msg::SensorTimestamp>::SharedPtr sensor1_subscriber_;   ///< Subscriber for Sensor 1 data
    rclcpp::Subscription<sensor_compute::msg::SensorTimestamp>::SharedPtr sensor2_subscriber_;   ///< Subscriber for Sensor 2 data
    rclcpp::TimerBase::SharedPtr save_timer_;   /// Enables saving 10s data snapshot
    std::vector<double> sensor1_data_;  ///< value for wave data for Sensor 1
    std::vector<double> sensor2_data_;  ///< value for wave data sensor 2
    std::vector<double> sensor1_latency_;   ///< latency values sensor 1 to calculate average
    std::vector<double> sensor2_latency_;   ///< latency values sensor 1 to calculate average
};

#endif  //CENTRAL_NODE
 
