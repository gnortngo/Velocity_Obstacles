#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <memory> 
#include <utility>
#include <thread>
#include <chrono>
#include "timer.h"

// Class RobotController
class RobotController {
public:
    RobotController(double radius, double height, double delta_t, double velocity, double initial_x, double initial_y)
        : radius_(radius), height_(height), delta_t_(delta_t), velocity_(velocity), current_x_(initial_x), current_y_(initial_y), current_index_(0) {}

    void set_goal_points(const std::vector<std::pair<double, double>>& points) {
        goal_points_ = points;
        current_index_ = 0;
        if (!goal_points_.empty()) {
            current_x_ = goal_points_[0].first;
            current_y_ = goal_points_[0].second;
        }
    }
    void move() {
        if (current_index_ >= goal_points_.size()) {
            return;
        }

        auto& goal = goal_points_[current_index_];
        double dx = goal.first - current_x_;
        double dy = goal.second - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance > velocity_ * delta_t_) {
            double angle = std::atan2(dy, dx);
            double velocity_x = velocity_ * std::cos(angle);
            double velocity_y = velocity_ * std::sin(angle);
            current_x_ += velocity_x * delta_t_;
            current_y_ += velocity_y * delta_t_;
            double remaining_distance = std::sqrt(std::pow(goal.first - current_x_, 2) +
                                                  std::pow(goal.second - current_y_, 2));
            if (remaining_distance < velocity_ * delta_t_) {
                current_x_ = goal.first;
                current_y_ = goal.second;
                ++current_index_;
            }
        } else {
            current_x_ = goal.first;
            current_y_ = goal.second;
            ++current_index_;
        }
    }

    double current_x() const { return current_x_; }
    double current_y() const { return current_y_; }
    double get_radius() const { return radius_; }
    double get_height() const { return height_; }

private:
    std::vector<std::pair<double, double>> goal_points_;
    double radius_;
    double height_;
    double delta_t_;
    double velocity_;
    double current_x_;
    double current_y_;
    size_t current_index_;
};

// Class MarkerPublisher
class MarkerPublisher {
public:
    MarkerPublisher(rclcpp::Node::SharedPtr node, std::shared_ptr<RobotController> robot_controller)
    : node_(node), robot_controller_(robot_controller) {
    publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    }

    void publish_marker() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = "robot_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        double x = robot_controller_->current_x(); 
        double y = robot_controller_->current_y(); 
        double height = robot_controller_->get_height(); 
        double radius = robot_controller_->get_radius(); 

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = height/2;  
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = radius;//0.5;
        marker.scale.y = radius;//0.5;
        marker.scale.z = height;//0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        publisher_->publish(marker);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<RobotController> robot_controller_;
    
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_controller_node");

    // Init
    auto robot1 = std::make_shared<RobotController>(0.5, 0.1, 0.1, 0.8, 0.0, 0.0); // r h delta_t v x0 y0
    auto robot2 = std::make_shared<RobotController>(0.7, 0.2, 0.1, 0.6, 1.0, 1.0);

    robot1->set_goal_points({         
            {0.0, 0.0},
            {0.0, 3.0},
            {3.0, 3.0},
            {3.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 3.0},
            {3.0, 3.0},
            {3.0, 0.0} 
        });

    robot2->set_goal_points({         
            {2.0, 0.0},
            {1.0, 5.0},
            {2.0, 3.0},
            {3.0, 1.0},
            {4.0, 0.0},
            {1.0, 0.0},
            {0.0, 0.0},
            {5.0, 5.0},
            {3.0, 3.0} 
        });

    auto marker_publisher1 = std::make_shared<MarkerPublisher>(node, robot1);
    auto marker_publisher2 = std::make_shared<MarkerPublisher>(node, robot2);

    auto control_robot = [](std::shared_ptr<RobotController> robot, std::shared_ptr<MarkerPublisher> publisher) {
        while (rclcpp::ok()) {
            robot->move();
            publisher->publish_marker();
            delay(100);
        }
    };

    std::thread robot1_thread(control_robot, robot1, marker_publisher1);
    std::thread robot2_thread(control_robot, robot2, marker_publisher2);

    robot1_thread.join();
    robot2_thread.join();

    rclcpp::shutdown();
    return 0;
}
