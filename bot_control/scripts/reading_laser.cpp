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
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float angle_min_ = 0.0f;
        float angle_max_ = 2.09f;

        RCLCPP_INFO(this->get_logger(), "Modified Angle Min: %f", angle_min_);
        RCLCPP_INFO(this->get_logger(), "Modified Angle Max: %f", angle_max_);

        RCLCPP_INFO(this->get_logger(), "Header: %d", msg->header.stamp.sec);
        RCLCPP_INFO(this->get_logger(), "Angle Increment: %f", msg->angle_increment);
        RCLCPP_INFO(this->get_logger(), "Range Min: %f", msg->range_min);
        RCLCPP_INFO(this->get_logger(), "Range Max: %f", msg->range_max);
        RCLCPP_INFO(this->get_logger(), "Ranges: size=%lu", msg->ranges.size());
        RCLCPP_INFO(this->get_logger(), "Intensities: size=%lu", msg->intensities.size());

        float angle_min = angle_min_;
        float angle_increment = msg->angle_increment;

        float start_angle = 0.0f;
        float end_angle = 2.09f;

        int start_index = static_cast<int>((start_angle - angle_min) / angle_increment);
        int end_index = static_cast<int>((end_angle - angle_min) / angle_increment);

        start_index = std::max(0, start_index);
        end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, end_index);

        RCLCPP_INFO(this->get_logger(), "Ranges from 0 to 2.09 radians:");
        for (int i = start_index; i <= end_index; ++i)
        {
            float angle = angle_min + i * angle_increment;
            RCLCPP_INFO(this->get_logger(), "Angle %.2f rad: Range %f", angle, msg->ranges[i]);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanValuesNode>());
    rclcpp::shutdown();
    return 0;
}
