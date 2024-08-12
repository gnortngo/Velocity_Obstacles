#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>

// Class publisher marker
class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher()
        : Node("marker_publisher")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    }

    void publish_marker(double x, double y)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "robot_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        publisher_->publish(marker);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

// Class move robot 
class RobotController : public rclcpp::Node
{
public:
    RobotController(const std::vector<std::pair<double, double>>& points)
        : Node("robot_controller"), 
          delta_t_(0.1), 
          velocity_(0.8), 
          current_x_(0.0), 
          current_y_(0.0), 
          current_index_(0),
          marker_publisher_(std::make_shared<MarkerPublisher>())
    {
        if (!points.empty()) {
            current_x_ = points[0].first;
            current_y_ = points[0].second;
            for (size_t i = 1; i < points.size(); ++i) {
                goal_points_.emplace_back(points[i].first, points[i].second);
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(delta_t_ * 1000)),
            std::bind(&RobotController::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Robot Controller Initialized");
    }

private:
    void timer_callback()
    {
        move();
        marker_publisher_->publish_marker(current_x_, current_y_);
        RCLCPP_INFO(this->get_logger(), "Robot position: (%f, %f)", current_x_, current_y_);
    }

    void move()
    {
        if (current_index_ >= goal_points_.size()) {
            return;
        }

        auto& goal = goal_points_[current_index_];
        double dx = goal.first - current_x_;
        double dy = goal.second - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance > velocity_*delta_t_) // Một giá trị ngưỡng
        {
            double angle = std::atan2(dy, dx);

            // Tính toán vận tốc trong x và y dựa trên vận tốc toàn phần
            double velocity_x = velocity_ * std::cos(angle);
            double velocity_y = velocity_ * std::sin(angle);

            // Cập nhật vị trí dựa trên công thức x(t) = x(t-1) + v * delta_t
            current_x_ += velocity_x * delta_t_;
            current_y_ += velocity_y * delta_t_;

            // Kiểm tra và điều chỉnh nếu robot vượt quá điểm mục tiêu
            double remaining_distance = std::sqrt(std::pow(goal.first - current_x_, 2) +
                                                  std::pow(goal.second - current_y_, 2));
            if (remaining_distance < velocity_ * delta_t_) {
                current_x_ = goal.first;
                current_y_ = goal.second;
                ++current_index_;
            }
        }
        else
        {
            current_x_ = goal.first;
            current_y_ = goal.second;
            ++current_index_;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double, double>> goal_points_;
    double delta_t_; // Thời gian giữa các lần gọi callback
    double velocity_; // Vận tốc của robot
    double current_x_;
    double current_y_;
    size_t current_index_;
    std::shared_ptr<MarkerPublisher> marker_publisher_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

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

    auto node = std::make_shared<RobotController>(points);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

