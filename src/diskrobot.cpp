#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

class DiskRobotPublisher : public rclcpp::Node
{
public:
    DiskRobotPublisher() : Node("disk_robot_publisher"),
        goal_x_(0.0), goal_y_(5.0), step_size_(0.01), current_x_(0.0), current_y_(0.0)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Cập nhật mỗi 100ms
            std::bind(&DiskRobotPublisher::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing robot marker");
    }

private:
    void timer_callback()
    {
        // Tính khoảng cách còn lại đến điểm đích
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Nếu chưa đến đích, di chuyển robot
        if (distance > step_size_)
        {
            // Tính toán góc và bước di chuyển
            double angle = std::atan2(dy, dx);
            current_x_ += step_size_ * std::cos(angle);
            current_y_ += step_size_ * std::sin(angle);
        }
        else
        {
            // Đạt đến đích
            current_x_ = goal_x_;
            current_y_ = goal_y_;
        }

        // Cập nhật marker
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";  // Đổi frame_id thành "map" để dễ dàng hiển thị trong RViz
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "robot_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_x_;
        marker.pose.position.y = current_y_;
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

        // Log thông tin
        RCLCPP_INFO(this->get_logger(), "Robot position: (%f, %f)", current_x_, current_y_);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double goal_x_;
    double goal_y_;
    double step_size_;
    double current_x_;
    double current_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiskRobotPublisher>());
    rclcpp::shutdown();
    return 0;
}

