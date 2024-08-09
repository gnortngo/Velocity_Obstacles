#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

class DiskRobotPublisher : public rclcpp::Node
{
public:
    DiskRobotPublisher() : Node("diskrobot")
    {
        // Declare parameters with default values
        this->declare_parameter("goal_x", 0.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("start_x", 0.0);
        this->declare_parameter("start_y", 0.0);

        // Initialize state variables
        goal_x_ = 0.0;
        goal_y_ = 0.0;
        current_x_ = 0.0;
        current_y_ = 0.0;
        step_size_ = 0.01;
        is_goal_set_ = false;

        // Create publisher and timer
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DiskRobotPublisher::timer_callback, this)
        );

        // Set up a parameter callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DiskRobotPublisher::parameters_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Node started. Please set start and goal parameters.");
    }

private:
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    void visualize()
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
        marker.header.frame_id = "map";
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
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        publisher_->publish(marker);


    void move()
    {
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance > step_size_)
        {
            double angle = std::atan2(dy, dx);
            current_x_ += step_size_ * std::cos(angle);
            current_y_ += step_size_ * std::sin(angle);
        }
        else
        {
            current_x_ = goal_x_;
            current_y_ = goal_y_;
            is_goal_set_ = false; // Stop moving when goal is reached
        }
    }

    void timer_callback()
    {
        if (is_goal_set_)
        {
            move();
            visualize();
            RCLCPP_INFO(this->get_logger(), "Robot position: (%f, %f)", current_x_, current_y_);
        }
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;  // Assume success unless an error is encountered

        for (const auto &param : parameters)
        {
            if (param.get_name() == "start_x")
            {
                current_x_ = param.as_double();
                current_y_ = this->get_parameter("start_y").as_double();
                visualize(); // Visualize the robot at the start position
                RCLCPP_INFO(this->get_logger(), "Start position set to: (%f, %f)", current_x_, current_y_);
            }
            else if (param.get_name() == "goal_x")
            {
                goal_x_ = param.as_double();
                goal_y_ = this->get_parameter("goal_y").as_double();
                is_goal_set_ = true; // Start moving towards the goal
                RCLCPP_INFO(this->get_logger(), "Goal position set to: (%f, %f)", goal_x_, goal_y_);
            }
            else
            {
                result.successful = false; // Parameter not recognized
                RCLCPP_WARN(this->get_logger(), "Parameter %s not recognized.", param.get_name().c_str());
            }
        }
        return result;

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
    bool is_goal_set_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiskRobotPublisher>());
    rclcpp::shutdown();
    return 0;
}

