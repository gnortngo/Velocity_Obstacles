#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
    RobotController(double radius, double height, double delta_t, double velocity, double initial_x, double initial_y, int robot_id)
        : radius_(radius), height_(height), delta_t_(delta_t), velocity_(velocity), current_x_(initial_x), current_y_(initial_y), current_index_(0), robot_id_(robot_id) {}

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
    int get_robot_id() const { return robot_id_; }

    double get_direction_angle() const {
        if (current_index_ >= goal_points_.size()) {
            return 0.0;
        }

        auto& goal = goal_points_[current_index_];
        double dx = goal.first - current_x_;
        double dy = goal.second - current_y_;
        return std::atan2(dy, dx);
    }

private:
    std::vector<std::pair<double, double>> goal_points_;
    double radius_;
    double height_;
    double delta_t_;
    double velocity_;
    double current_x_;
    double current_y_;
    size_t current_index_;
    int robot_id_;
};

// Class MarkerPublisher
class MarkerPublisher {
public:
    MarkerPublisher(rclcpp::Node::SharedPtr node, std::shared_ptr<RobotController> robot_controller)
    : node_(node), robot_controller_(robot_controller) {
    publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 100);
    }

    void publish_marker() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = "robot_marker";
        marker.id = robot_controller_->get_robot_id(); 
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
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = height;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = rclcpp::Duration(1, 0); 
         marker_array.markers.push_back(marker);
        
                // Marker for direction arrow
        auto arrow_marker = visualization_msgs::msg::Marker();
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = marker.header.stamp;
        arrow_marker.ns = "robot_marker_arrow";
        arrow_marker.id = robot_controller_->get_robot_id() + 1000;  // Ensure unique ID for arrow marker
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // Arrow position and orientation
        arrow_marker.pose.position.x = x;
        arrow_marker.pose.position.y = y;
        arrow_marker.pose.position.z = height / 2;
        
        double direction_angle = robot_controller_->get_direction_angle();
        arrow_marker.pose.orientation.x = 0.0;
        arrow_marker.pose.orientation.y = 0.0;
        arrow_marker.pose.orientation.z = sin(direction_angle / 2);
        arrow_marker.pose.orientation.w = cos(direction_angle / 2);

        // Arrow size
        arrow_marker.scale.x = 0.6; // Arrow length
        arrow_marker.scale.y = 0.06; // Arrow width
        arrow_marker.scale.z = 0.06; // Arrow height

        arrow_marker.color.a = 1.0;
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;
        marker_array.markers.push_back(arrow_marker);

        publisher_->publish(marker_array);
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<RobotController> robot_controller_;
    
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_controller_node");

    // Init
    auto robot1 = std::make_shared<RobotController>(0.4, 0.1, 0.1, 1.0, 0.0, 0.0, 1); // r h delta_t v x0 y0 id
    auto robot2 = std::make_shared<RobotController>(0.4, 0.1, 0.1, 0.8, 1.0, 1.0, 2);
    auto robot3 = std::make_shared<RobotController>(0.4, 0.1, 0.1, 0.6, 1.0, 1.0, 3);
    auto robot4 = std::make_shared<RobotController>(0.4, 0.1, 0.1, 0.4, 1.0, 1.0, 4);
    auto robot5 = std::make_shared<RobotController>(0.4, 0.1, 0.1, 0.2, 1.0, 1.0, 5);
    robot1->set_goal_points({         
            {5.00, 0.00}, 
            {4.05, 2.94}, 
            {1.54, 4.75}, 
            {-1.54, 4.75}, 
            {-4.05, 2.94}, 
            {-5.00, 0.00}, 
            {-4.05, -2.94}, 
            {-1.54, -4.75}, 
            {1.54, -4.75}, 
            {4.05, -2.94}
        });

    robot2->set_goal_points({         
            {4.00, 0.00}, 
            {3.24, 2.35}, 
            {1.23, 3.80}, 
            {-1.23, 3.80}, 
            {-3.24, 2.35}, 
            {-4.00, 0.00}, 
            {-3.24, -2.35}, 
            {-1.23, -3.80}, 
            {1.23, -3.80}, 
            {3.24, -2.35}
        });
    robot3->set_goal_points({         
            {3.00, 0.00}, 
            {2.43, 1.76}, 
            {0.92, 2.85}, 
            {-0.92, 2.85}, 
            {-2.43, 1.76}, 
            {-3.00, 0.00}, 
            {-2.43, -1.76}, 
            {-0.92, -2.85}, 
            {0.92, -2.85}, 
            {2.43, -1.76}
        });
    robot4->set_goal_points({         
            {2.00, 0.00}, 
            {1.62, 1.17}, 
            {0.61, 1.90}, 
            {-0.61, 1.90}, 
            {-1.62, 1.17}, 
            {-2.00, 0.00}, 
            {-1.62, -1.17}, 
            {-0.61, -1.90}, 
            {0.61, -1.90}, 
            {1.62, -1.17}
        });
    robot5->set_goal_points({         
            {1.00, 0.00}, 
            {0.81, 0.59}, 
            {0.31, 0.95}, 
            {-0.31, 0.95}, 
            {-0.81, 0.59}, 
            {-1.00, 0.00}, 
            {-0.81, -0.59}, 
            {-0.31, -0.95}, 
            {0.31, -0.95}, 
            {0.81, -0.59}
        });
    auto marker_publisher1 = std::make_shared<MarkerPublisher>(node, robot1);
    auto marker_publisher2 = std::make_shared<MarkerPublisher>(node, robot2);
    auto marker_publisher3 = std::make_shared<MarkerPublisher>(node, robot3);
    auto marker_publisher4 = std::make_shared<MarkerPublisher>(node, robot4);
    auto marker_publisher5 = std::make_shared<MarkerPublisher>(node, robot5);
    // Main loop
    rclcpp::WallRate loop_rate(5); // 10 Hz loop rate
    while (rclcpp::ok()) {
        robot1->move();
        robot2->move();
        robot3->move();
        robot4->move();
        robot5->move();
        marker_publisher1->publish_marker();
        marker_publisher2->publish_marker();
        marker_publisher3->publish_marker();
        marker_publisher4->publish_marker();
        marker_publisher5->publish_marker();

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
