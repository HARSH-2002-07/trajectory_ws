/**
 * @file trajectory_controller_node.cpp
 * @brief ROS2 node for trajectory tracking control using Pure Pursuit
 * 
 * This node implements a complete trajectory tracking system for differential
 * drive robots, including path smoothing, trajectory generation, and control.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

/**
 * @brief Structure to represent a 2D point
 */
struct Point2D {
    double x;
    double y;
    
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    double distance_to(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

/**
 * @brief Structure to represent robot pose (position + orientation)
 */
struct Pose2D {
    double x;
    double y;
    double theta;
    
    Pose2D() : x(0.0), y(0.0), theta(0.0) {}
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * @brief Path smoother using Catmull-Rom splines
 * 
 * Provides C1 continuity for smooth paths suitable for robot navigation.
 */
class PathSmoother {
public:
    /**
     * @brief Smooth a path using Catmull-Rom interpolation
     * @param waypoints Input waypoints
     * @param points_per_segment Number of points between each waypoint pair
     * @return Smoothed path as vector of points
     */
    static std::vector<Point2D> smooth_path(
        const std::vector<Point2D>& waypoints,
        int points_per_segment = 20) {
        
        if (waypoints.size() < 2) {
            return waypoints;
        }
        
        if (waypoints.size() == 2) {
            // Linear interpolation for two points
            return linear_interpolation(waypoints[0], waypoints[1], points_per_segment);
        }
        
        std::vector<Point2D> smooth_path;
        
        // Add first waypoint
        smooth_path.push_back(waypoints[0]);
        
        // Catmull-Rom spline for each segment
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            Point2D p0 = (i == 0) ? waypoints[0] : waypoints[i - 1];
            Point2D p1 = waypoints[i];
            Point2D p2 = waypoints[i + 1];
            Point2D p3 = (i + 2 < waypoints.size()) ? waypoints[i + 2] : waypoints[i + 1];
            
            // Generate points along spline
            for (int j = 1; j <= points_per_segment; ++j) {
                double t = static_cast<double>(j) / points_per_segment;
                Point2D pt = catmull_rom_point(p0, p1, p2, p3, t);
                smooth_path.push_back(pt);
            }
        }
        
        return smooth_path;
    }

private:
    /**
     * @brief Compute point on Catmull-Rom spline
     */
    static Point2D catmull_rom_point(
        const Point2D& p0, const Point2D& p1,
        const Point2D& p2, const Point2D& p3,
        double t) {
        
        double t2 = t * t;
        double t3 = t2 * t;
        
        double x = 0.5 * ((2.0 * p1.x) +
                         (-p0.x + p2.x) * t +
                         (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
                         (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);
        
        double y = 0.5 * ((2.0 * p1.y) +
                         (-p0.y + p2.y) * t +
                         (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
                         (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
        
        return Point2D(x, y);
    }
    
    /**
     * @brief Linear interpolation between two points
     */
    static std::vector<Point2D> linear_interpolation(
        const Point2D& p1, const Point2D& p2, int num_points) {
        
        std::vector<Point2D> result;
        for (int i = 0; i <= num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            double x = p1.x + t * (p2.x - p1.x);
            double y = p1.y + t * (p2.y - p1.y);
            result.push_back(Point2D(x, y));
        }
        return result;
    }
};

/**
 * @brief Pure Pursuit controller for trajectory tracking
 * 
 * Implements the Pure Pursuit algorithm for smooth path following
 * on differential drive robots.
 */
class PurePursuitController {
public:
    PurePursuitController(
        double lookahead_distance = 0.5,
        double max_linear_vel = 0.5,
        double max_angular_vel = 2.0,
        double goal_tolerance = 0.1)
        : lookahead_distance_(lookahead_distance),
          max_linear_vel_(max_linear_vel),
          max_angular_vel_(max_angular_vel),
          goal_tolerance_(goal_tolerance) {}
    
    /**
     * @brief Compute velocity commands for trajectory tracking
     * @param current_pose Current robot pose
     * @param path Target path to follow
     * @param[out] linear_vel Computed linear velocity
     * @param[out] angular_vel Computed angular velocity
     * @return true if goal reached, false otherwise
     */
    bool compute_control(
        const Pose2D& current_pose,
        const std::vector<Point2D>& path,
        double& linear_vel,
        double& angular_vel) {
        
        if (path.empty()) {
            linear_vel = 0.0;
            angular_vel = 0.0;
            return true;
        }
        
        // Check if goal reached
        Point2D goal = path.back();
        double dist_to_goal = std::hypot(goal.x - current_pose.x, goal.y - current_pose.y);
        
        if (dist_to_goal < goal_tolerance_) {
            linear_vel = 0.0;
            angular_vel = 0.0;
            return true;
        }
        
        // Find lookahead point
        Point2D lookahead_point;
        if (!find_lookahead_point(current_pose, path, lookahead_point)) {
            // No valid lookahead point, aim for goal
            lookahead_point = goal;
        }
        
        // Compute angle to lookahead point
        double dx = lookahead_point.x - current_pose.x;
        double dy = lookahead_point.y - current_pose.y;
        double angle_to_target = std::atan2(dy, dx);
        
        // Compute heading error
        double angle_error = normalize_angle(angle_to_target - current_pose.theta);
        
        // Pure Pursuit control law
        double distance = std::hypot(dx, dy);
        if (distance < 1e-6) {
            linear_vel = 0.0;
            angular_vel = 0.0;
            return false;
        }
        
        // Curvature calculation
        double curvature = 2.0 * std::sin(angle_error) / distance;
        
        // Compute velocities
        linear_vel = max_linear_vel_;
        angular_vel = curvature * linear_vel;
        
        // Apply velocity limits
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        // Reduce linear velocity for sharp turns
        if (std::abs(angular_vel) > 0.5) {
            double reduction_factor = 1.0 - (std::abs(angular_vel) / max_angular_vel_) * 0.5;
            linear_vel *= std::max(reduction_factor, 0.3);
        }
        
        return false;
    }
    
    void set_lookahead_distance(double distance) { lookahead_distance_ = distance; }
    void set_max_linear_vel(double vel) { max_linear_vel_ = vel; }
    void set_max_angular_vel(double vel) { max_angular_vel_ = vel; }

private:
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;
    
    /**
     * @brief Find lookahead point on path
     * @param pose Current robot pose
     * @param path Target path
     * @param[out] lookahead_point Found lookahead point
     * @return true if valid lookahead point found
     */
    bool find_lookahead_point(
        const Pose2D& pose,
        const std::vector<Point2D>& path,
        Point2D& lookahead_point) {
        
        // Find closest point on path
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < path.size(); ++i) {
            double dist = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Search forward for lookahead point
        for (size_t i = closest_idx; i < path.size(); ++i) {
            double dist = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
            
            if (dist >= lookahead_distance_) {
                lookahead_point = path[i];
                return true;
            }
        }
        
        // Return last point if no lookahead found
        if (!path.empty()) {
            lookahead_point = path.back();
            return true;
        }
        
        return false;
    }
    
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

/**
 * @brief ROS2 node for trajectory tracking
 */
class TrajectoryControllerNode : public rclcpp::Node {
public:
    TrajectoryControllerNode() : Node("trajectory_controller_node") {
        // Declare parameters
        this->declare_parameter("lookahead_distance", 0.5);
        this->declare_parameter("max_linear_velocity", 0.5);
        this->declare_parameter("max_angular_velocity", 2.0);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("points_per_segment", 20);
        
        // Get parameters
        double lookahead = this->get_parameter("lookahead_distance").as_double();
        double max_lin_vel = this->get_parameter("max_linear_velocity").as_double();
        double max_ang_vel = this->get_parameter("max_angular_velocity").as_double();
        double goal_tol = this->get_parameter("goal_tolerance").as_double();
        double freq = this->get_parameter("control_frequency").as_double();
        points_per_segment_ = this->get_parameter("points_per_segment").as_int();
        
        // Initialize controller
        controller_ = std::make_unique<PurePursuitController>(
            lookahead, max_lin_vel, max_ang_vel, goal_tol);
        
        // Create subscribers
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "waypoints", 10,
            std::bind(&TrajectoryControllerNode::waypoint_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&TrajectoryControllerNode::odometry_callback, this, std::placeholders::_1));
        
        // Create publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("smooth_path", 10);
        
        // Create control timer
        auto timer_period = std::chrono::duration<double>(1.0 / freq);
        control_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&TrajectoryControllerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Trajectory Controller Node initialized");
        RCLCPP_INFO(this->get_logger(), "Lookahead distance: %.2f m", lookahead);
        RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_lin_vel);
        RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", freq);
    }

private:
    // ROS2 components
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Controller and state
    std::unique_ptr<PurePursuitController> controller_;
    std::vector<Point2D> current_path_;
    Pose2D current_pose_;
    bool odom_received_ = false;
    bool goal_reached_ = false;
    int points_per_segment_;
    
    /**
     * @brief Callback for waypoint messages
     */
    void waypoint_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty waypoint array");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", msg->poses.size());
        
        // Convert waypoints to Point2D
        std::vector<Point2D> waypoints;
        for (const auto& pose : msg->poses) {
            waypoints.push_back(Point2D(pose.position.x, pose.position.y));
        }
        
        // Smooth the path
        current_path_ = PathSmoother::smooth_path(waypoints, points_per_segment_);
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Generated smooth path with %zu points", 
                    current_path_.size());
        
        // Publish smooth path for visualization
        publish_smooth_path();
    }
    
    /**
     * @brief Callback for odometry messages
     */
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;
        
        odom_received_ = true;
    }
    
    /**
     * @brief Main control loop
     */
    void control_loop() {
        if (!odom_received_) {
            return;
        }
        
        if (current_path_.empty() || goal_reached_) {
            // No path or goal reached - stop robot
            publish_zero_velocity();
            return;
        }
        
        // Compute control commands
        double linear_vel, angular_vel;
        bool goal_reached = controller_->compute_control(
            current_pose_, current_path_, linear_vel, angular_vel);
        
        if (goal_reached) {
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            publish_zero_velocity();
            return;
        }
        
        // Publish velocity commands
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_vel;
        twist_msg.angular.z = angular_vel;
        cmd_vel_pub_->publish(twist_msg);
    }
    
    /**
     * @brief Publish smooth path for visualization
     */
    void publish_smooth_path() {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom";
        
        for (const auto& point : current_path_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose_stamped);
        }
        
        smooth_path_pub_->publish(path_msg);
    }
    
    /**
     * @brief Publish zero velocity command
     */
    void publish_zero_velocity() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg);
    }
};

/**
 * @brief Main function
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}