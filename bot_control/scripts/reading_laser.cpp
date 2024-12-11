#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanValuesNode : public rclcpp::Node
{
public:
    ScanValuesNode() : Node("scan_values")
    {
        
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",  
            10,      
            std::bind(&ScanValuesNode::callback, this, std::placeholders::_1)
        );

        filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const float start_angle = -60.0f * (M_PI / 180.0f);  
        const float end_angle = 60.0f * (M_PI / 180.0f);    
        const float obstacle_threshold = 5.0f;              

        
        int start_index = static_cast<int>((start_angle - msg->angle_min) / msg->angle_increment);
        int end_index = static_cast<int>((end_angle - msg->angle_min) / msg->angle_increment);

        
        start_index = std::max(0, start_index);
        end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, end_index);

        bool obstacle_detected = false;
        
        
        sensor_msgs::msg::LaserScan filtered_scan = *msg;

       
        for (int i = start_index; i <= end_index; ++i)
        {
            if (msg->ranges[i] < obstacle_threshold && msg->ranges[i] >= msg->range_min)
            {
                float angle = msg->angle_min + i * msg->angle_increment;
                RCLCPP_INFO(this->get_logger(), "Obstacle detected at Angle: %.2f rad, Range: %.2f m", angle, msg->ranges[i]);
                obstacle_detected = true;
                filtered_scan.ranges[i] = msg->ranges[i]; 
            }
            else
            {
                filtered_scan.ranges[i] = msg->range_max;  
            }

            
            float angle = msg->angle_min + i * msg->angle_increment;
            RCLCPP_INFO(this->get_logger(), "Angle: %.2f rad, Range: %.2f m", angle, filtered_scan.ranges[i]);
        }

      
        if (!obstacle_detected)
        {
            RCLCPP_INFO(this->get_logger(), "No obstacle detected from -60° to +60°. Range: Infinity");
            for (int i = start_index; i <= end_index; ++i)
            {
                filtered_scan.ranges[i] = msg->range_max;  
            }
        }

        
        filtered_scan_publisher_->publish(filtered_scan);
    }

    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanValuesNode>());
    rclcpp::shutdown();
    return 0;
}
