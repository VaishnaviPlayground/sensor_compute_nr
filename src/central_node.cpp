#include "central_node.hpp"
#include <fstream>

using sensor_type = sensor_compute::msg::SensorTimestamp;

CentralNode::CentralNode() : Node("central_node") 
{   
    // Subscriber initialized for sensor 1
    sensor1_subscriber_ = this->create_subscription<sensor_type>("sensor1_topic", 10, std::bind(&CentralNode::sensor1_subscribe, this, std::placeholders::_1));
    // Subscriber initialized for sensor 2    
    sensor2_subscriber_ = this->create_subscription<sensor_type>("sensor2_topic", 10, std::bind(&CentralNode::sensor2_subscribe, this, std::placeholders::_1));
    // Timer for snapshots to be saved every 10 seconds
    save_timer_ = this->create_wall_timer(std::chrono::seconds(10),std::bind(&CentralNode::wave_sensor_data, this));
}


void CentralNode::sensor1_subscribe(const sensor_type::SharedPtr msg) 
{
    // sensor 1 subscribed measure latencies for 1st sensor
    latency_measurement("sensor1", msg);
    // pushing the wava data value in vector to plot later
    sensor1_data_.push_back(msg->data);
}

void CentralNode::sensor2_subscribe(const sensor_type::SharedPtr msg) 
{
    // sensor 2 subscribed measure latencies for 2nd sensor
    latency_measurement("sensor2", msg);
    // pushing the wava data value in vector to plot later
    sensor2_data_.push_back(msg->data);
}

void CentralNode::latency_measurement(const std::string &sensor_name, const sensor_type::SharedPtr msg) 
{
    // Using time from rclcpp to make sure time in synchronized
    rclcpp::Time current_time = this->now();
    // Calculate latency and conver in milliseconds
    double latency= (current_time - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
    // Log latencies to be seen when node runs
    RCLCPP_INFO(this->get_logger(), " %s subscribed and the latency is: %f ms", sensor_name.c_str(), latency);
    // logic to see if sensor 1 is subscribed or 2 and based on that push bach in vector so i can measure average latencies every 10 seconds
    if (sensor_name == "sensor1") 
    {
      sensor1_latency_.push_back(latency);
    } 
    else if (sensor_name == "sensor2") 
    {
      sensor2_latency_.push_back(latency);
    }

}

double CentralNode::compute_average_latency(const std::vector<double>& latencies) 
{   
     double sum = 0.0;
    // check if its empty
    if (latencies.empty()) 
    {
       // return nothing      
        return 0.0;
    }
    // compute average
    for (double latency : latencies) 
    {
        sum = sum +latency;
    }
    // return average
    return sum / latencies.size();
}


void CentralNode::wave_sensor_data() 
{
    // Open files in append for writing sensor data and latency measurements
    // TODO: better saving strategy, should i only save once
    std::ofstream sensor_data_file("sensor_data.csv", std::ios::app);
    std::ofstream latency_file("latency_measurements.txt", std::ios::app);
    static bool firstrow = 0;
    // CSV file first row to create columns
    if (!firstrow) {
        sensor_data_file << "Time,Sensor1,Sensor2\n";
        firstrow = 1;
    }

    // Write sensor data to file
    for (size_t i = 0; i < sensor1_data_.size(); ++i) 
    {
        // assuming my publishing happens correctly
        //TODO: is it really 2ms? Linux isnt that powerful always
        double timescale = i * 0.002;
        // seperated by comma in my csv
        sensor_data_file << timescale << "," << sensor1_data_[i] << "," << sensor2_data_[i] << "\n";
    }

    // Compute and log average latency for both sensors
    double avg_latency1 = compute_average_latency(sensor1_latency_);
    double avg_latency2 = compute_average_latency(sensor2_latency_);
    
    latency_file << "Average Latency (ms):\n"
                 << "  Sensor1: " << avg_latency1 << " ms\n"
                 << "  Sensor2: " << avg_latency2 << " ms\n";

    // Clear stored data to prepare for the next cycle
    sensor1_data_.clear();
    sensor2_data_.clear();
    sensor1_latency_.clear();
    sensor2_latency_.clear();
    sensor_data_file.close();
    latency_file.close();
}

int main(int argc, char **argv) 
{
    //init
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CentralNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
