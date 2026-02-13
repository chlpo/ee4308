#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        // initialize topics
        // this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "scan", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    // void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    // {
    //     this->scan_ranges_ = msg->ranges;
    // }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        

        
        


        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        double goal_pose_x = goal_pose.pose.position.x;
        double goal_pose_y = goal_pose.pose.position.y;
        //print(goal_pose_x);

        //constant, robot need to stop if smaller than this 
        if (std::pow(goal_pose_x - rbt_pose.pose.position.x,2)+ std::pow(goal_pose_y- rbt_pose.pose.position.y,2) < std::pow(this->xy_goal_thres_,2)) {
            return writeCmdVel(0,0);
        }

        //global_plan_.poses.next();

        //transform lookahead point to global frame from robot frame

        //find closest pose
        geometry_msgs::msg::PoseStamped closest_pose;
        size_t i = 0;
        double min_distance = std::numeric_limits<double>::infinity();  // Start with a very large distance

        while (i < global_plan_.poses.size()) {
            // Calculate the Euclidean distance between the robot and the current pose
            double x_diff = global_plan_.poses[i].pose.position.x - rbt_pose.pose.position.x;
            double y_diff = global_plan_.poses[i].pose.position.y - rbt_pose.pose.position.y;
            double distance = std::sqrt(x_diff * x_diff + y_diff * y_diff);

            // If this distance is smaller than the previous minimum, update the closest pose
            if (distance < min_distance) {
                min_distance = distance;
                closest_pose = global_plan_.poses[i];
    }

        i++;
}

        size_t j = 0;
        // get lookahead from closest? based on desired distance
        geometry_msgs::msg::PoseStamped lookahead_pose;// = goal_pose; 
        while(j < global_plan_.poses.size()){
            //check the for the closest pose based on desired look ahead distance
            geometry_msgs::msg::PoseStamped temp_pose = global_plan_.poses.at(j);
            double x_c = temp_pose.pose.position.x - closest_pose.pose.position.x;
            double y_c = temp_pose.pose.position.y - closest_pose.pose.position.y;
            double hyp = std::hypot(x_c,y_c);
            if (hyp > desired_lookahead_dist_){
                lookahead_pose = temp_pose;
                break;
            }
            j++;
        }
        
        //double linear_vel = 0 * (lookahead_pose.pose.position.x - rbt_pose.pose.position.x);
        //double angular_vel = 0 * ee4308::getYawFromQuaternion(goal_pose.pose.orientation);

        //geometry_msgs::msg::PoseStamped transformed_lookahead_pose;
        //transformed_lookahead_pose = tf_->transform(rbt_pose_odom, transformed_lookahead_pose, "map");;

        //rest of the algorithm
        //method 1
        double x_change = lookahead_pose.pose.position.x - rbt_pose.pose.position.x;
        double y_change = lookahead_pose.pose.position.y - rbt_pose.pose.position.y;

        //double d = std::hypot(x_change,y_change);
        
        //double heading_angle = 2*(std::acos(rbt_pose.pose.orientation.w)); //quarternion angle derivation
        double heading_angle = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);


        double x_prime = x_change*std::cos(heading_angle)+y_change*std::sin(heading_angle); //prime is the lookahead point
        double y_prime = y_change*std::cos(heading_angle)-x_change*std::sin(heading_angle);

        //curvature code
        

        double linear_vel = desired_linear_vel_;
        double angular_vel;

        double curvature = (2*y_prime)/((std::pow(x_prime,2))+ std::pow(y_prime,2));

        //w = vc
        angular_vel = linear_vel*curvature;

        if(angular_vel > max_angular_vel_){
            angular_vel = max_angular_vel_;
        }

        if(linear_vel > max_linear_vel_){
            linear_vel = max_linear_vel_;
        }

        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "odom";
        cmd_vel.header.stamp = this->node_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    // ======================================== DO NOT TOUCH =================================

    void Controller::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { this->global_plan_ = path; }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)