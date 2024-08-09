#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>

class DiskRobotPublisher : public rclcpp::Node
{
public:
    // Constructor nhận tham số là danh sách các điểm đích
    DiskRobotPublisher(const std::vector<std::pair<double, double>>& points)
        : Node("disk_robot_publisher"), step_size_(0.08), current_index_(0)
    {
        // Khởi tạo tọa độ điểm bắt đầu và danh sách điểm đích
        if (!points.empty()) {
            current_x_ = points[0].first;
            current_y_ = points[0].second;
            for (size_t i = 1; i < points.size(); ++i) {
                goal_points_.emplace_back(points[i].first, points[i].second);
            }
        }

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
        move();
        visualize();
        // Log thông tin
        RCLCPP_INFO(this->get_logger(), "Robot position: (%f, %f)", current_x_, current_y_);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void move()
    {
        if (current_index_ >= goal_points_.size()) {
            // Nếu đã đi qua tất cả các điểm, không di chuyển thêm
            return;
        }

        auto& goal = goal_points_[current_index_];

        // Tính khoảng cách còn lại đến điểm đích hiện tại
        double dx = goal.first - current_x_;
        double dy = goal.second - current_y_;
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
            // Đạt đến đích hiện tại, chuyển sang điểm đích tiếp theo
            current_x_ = goal.first;
            current_y_ = goal.second;
            ++current_index_;
        }
    }
    
    void visualize()
    {
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
    }

    std::vector<std::pair<double, double>> goal_points_;
    double step_size_;
    double current_x_;
    double current_y_;
    size_t current_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Tạo danh sách các điểm đích
    std::vector<std::pair<double, double>> points = {
        {0.0, 0.0},
        {0.0, 3.0},
        {3.0, 3.0},
        {3.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 3.0},
        {3.0, 3.0},
        {3.0, 0.0}
    };

    // Tạo đối tượng của DiskRobotPublisher với danh sách điểm đích
    auto node = std::make_shared<DiskRobotPublisher>(points);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

