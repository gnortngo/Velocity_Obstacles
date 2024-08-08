#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class DiskRobotPublisher : public rclcpp::Node
{
public:
    DiskRobotPublisher() : Node("diskrobot")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DiskRobotPublisher::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing robot marker");
    }

private:
    void timer_callback()
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "robot_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;  // Radius
        marker.scale.y = 0.5;  // Radius
        marker.scale.z = 0.1;  // Height
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        publisher_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiskRobotPublisher>());
    rclcpp::shutdown();
    return 0;
}

