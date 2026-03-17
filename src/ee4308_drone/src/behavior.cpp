#include "ee4308_drone/behavior.hpp"

namespace ee4308::drone
{

    Behavior::Behavior(
        const rclcpp::NodeOptions &options,
        const std::string &name = "behavior")
        : Node(name, options)
    {
        // parameters
        this->reached_thres_ = ee4308::getParameter<double>(this, "reached_thres", 0.2).as_double();
        this->cruise_height_ = ee4308::getParameter<double>(this, "cruise_height", 4.0).as_double();
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 5.0).as_double();
        this->topic_turtle_plan_ = ee4308::getParameter<std::string>(this, "topic_turtle_plan", "/turtle/plan").as_string();
        this->topic_turtle_stop_ = ee4308::getParameter<std::string>(this, "topic_turtle_stop", "/turtle/stop").as_string();
        this->frame_id_map_ = ee4308::getParameter<std::string>(this, "map_frame_id", "map").as_string();
        this->initial_x_ = ee4308::getParameter<double>(this, "initial_x", -2.0).as_double();
        this->initial_y_ = ee4308::getParameter<double>(this, "initial_y", -2.0).as_double();
        this->initial_z_ = ee4308::getParameter<double>(this, "initial_z", 0.05).as_double();

        // topics
        this->sub_est_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
                                                                                 std::bind(&Behavior::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_turtle_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            this->topic_turtle_stop_, rclcpp::ServicesQoS(),
            std::bind(&Behavior::callbackSubTurtleStop_, this, std::placeholders::_1));
        this->sub_turtle_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            this->topic_turtle_plan_, rclcpp::SensorDataQoS(),
            std::bind(&Behavior::callbackSubTurtlePlan_, this, std::placeholders::_1));
        this->pub_enable_ = this->create_publisher<std_msgs::msg::Bool>("enable", rclcpp::ServicesQoS());

        // services
        this->cli_plan_ = this->create_client<nav_msgs::srv::GetPlan>("get_plan");

        // states
        this->turtle_stop_ = false;
        this->plan_requested_ = false;
        this->state_ = BEGIN;
        this->transition_(TAKEOFF);
        this->timer_ = this->create_timer(1s / frequency_, std::bind(&Behavior::callbackTimer_, this));
    }

    void Behavior::callbackSubOdom_(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->odom_ = *msg;
    }

    void Behavior::callbackSubTurtleStop_(std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        this->turtle_stop_ = true;
    }

    void Behavior::callbackSubTurtlePlan_(nav_msgs::msg::Path::SharedPtr msg)
    {
        this->turtle_plan_ = *msg;
    }

    void Behavior::callbackTimer_()
    {
        // ==== make use of ====
        // state_
        // TAKEOFF, TURTLE_POSITION, TURTLE_WAYPOINT, INITIAL, LANDING, END
        // odom_
        // waypoint_x_, waypoint_y_, waypoint_z_
        // ==== ====

        //int fsm[7][7] ;
        double x_diff = odom_.pose.pose.position.x - waypoint_x_;
        double y_diff = odom_.pose.pose.position.y - waypoint_y_;
        double z_diff = odom_.pose.pose.position.z - waypoint_z_;

        double magnitude = std::pow((std::pow(x_diff,2)+std::pow(y_diff,2)+std::pow(z_diff,2)),0.5);


        if (magnitude <= reached_thres_)  // reached waypoint
        {
            /*if (state_ == TAKEOFF)
            {
                transition_(INITIAL);
            }*/
            // ...
            switch(state_){
                case BEGIN: //0
                    transition_(TAKEOFF);
                    break;
                case TAKEOFF: //1
                    if(odom_.pose.pose.position.z > cruise_height_){
                        transition_(TURTLE_POSITION);  
                    } else{ //not at cruise height yet
                        transition_(INITIAL);}
                    break;
                case TURTLE_POSITION://2
                    transition_(TURTLE_WAYPOINT);
                    break;

                case TURTLE_WAYPOINT://3
                    if(turtle_stop_){//if turtle has stopped --- then how? check if 3 states are reached?
                        transition_(LANDING);
                    } else {
                        transition_(INITIAL);
                    }
                    break;
                case INITIAL: //4
                    if(turtle_stop_){ //if turtle has stopped and reached goal
                        transition_(LANDING);
                    } else {
                        transition_(TURTLE_POSITION);}
                    break;
                case LANDING://5
                        if (odom_.twist.twist.angular.z == 0 && odom_.twist.twist.angular.y == 0 && odom_.twist.twist.angular.x == 0 
                            && odom_.twist.twist.linear.z == 0 && odom_.twist.twist.linear.y == 0 && odom_.twist.twist.linear.x ==0
                        ){
                            transition_(END);
                        }
                        break;
                        
            

            }
        } else {
            Behavior::requestPlan_(odom_.pose.pose.position.x, odom_.pose.pose.position.y,odom_ .pose.pose.position.z,
            waypoint_x_, waypoint_y_, waypoint_z_);
        }

        // request a plan with requestPlan(). This is done every time cbTimer() is called.
        // ...
    }

    void Behavior::transition_(int new_state)
    {
        // std::cout << "transition_ from " << state_ << " To " << new_state << std::endl;

        // ==== make use of ====
        // new_state (function argument)
        // state_
        // TAKEOFF, TURTLE_POSITION, TURTLE_WAYPOINT, INITIAL, LANDING, END
        // initial_x_, initial_y_, initial_z_,
        // cruise_height_
        // turtle_plan_.poses
        // turtle_stop_
        // =========

        // modify the following
        state_ = new_state;

        if (state_ == TAKEOFF)
        {
            // turn on the robot
            std_msgs::msg::Bool msg_enable;
            msg_enable.data = true;
            this->pub_enable_->publish(msg_enable);

            // set the waypoint.
            setWaypoint_(initial_x_, initial_y_, cruise_height_);
            //if initial remains lower than cruise height

            
        } else if (state_ == TURTLE_WAYPOINT){
            //check if its at the goal yet
            geometry_msgs::msg::PoseStamped_ goal = turtle_plan_.poses.back();
            if(goal.pose.position.x == odom_.pose.pose.position.x && goal.pose.position.y == odom_.pose.pose.position.y && goal.pose.position.z == odom_.pose.pose.position.z){
                turtle_stop_ = true;
            } else {
                //go to current waypoint ??
            }
            
        }else if (state_ == LANDING){
            odom_.pose.pose.position.x = initial_x_;
            odom_.pose.pose.position.y = initial_y_;
            odom_.pose.pose.position.z = cruise_height_;
        }
        else if (state_ == END)
        {
            // turn off the robot
            std_msgs::msg::Bool msg_enable;
            msg_enable.data = false;
            this->pub_enable_->publish(msg_enable);

            // delete the timer to stop the state machine.
            this->timer_->cancel();
            this->timer_ = nullptr; 
        }
    }

    void Behavior::setWaypoint_(double waypoint_x, double waypoint_y, double waypoint_z)
    {
        // you may choose to not use this function.
        // ==== make use of ====
        // waypoint_x_, waypoint_y_, waypoint_z_
        // waypoint_x, waypoint_y, waypoint_z
        // =========
        
        // remove or rewrite the following
        (void) waypoint_x; // remove or .
        (void) waypoint_y; // write to private class property.
        (void) waypoint_z; // write to private class property.
    }

    bool Behavior::reachedWaypoint_()
    {
        // you may choose to not use this function.
        // returns true if the current waypoint is reached.
        
        // remove or rewrite the following.
        return true;
    }

    void Behavior::requestPlan_(double drone_x, double drone_y, double drone_z,
                                double waypoint_x, double waypoint_y, double waypoint_z)
    { // Sends a non-blocking service request to publish a path from the planner node.
        if (plan_requested_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No request is made as there is no response yet from previous request.");
            return;
        }
        plan_requested_ = true;
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->goal.header.frame_id = this->frame_id_map_;
        request->goal.header.stamp = this->now();
        request->goal.pose.position.x = waypoint_x;
        request->goal.pose.position.y = waypoint_y;
        request->goal.pose.position.z = waypoint_z;
        request->start.header.frame_id = this->frame_id_map_;
        request->start.header.stamp = this->now();
        request->start.pose.position.x = drone_x;
        request->start.pose.position.y = drone_y;
        request->start.pose.position.z = drone_z;
        request_plan_future_ = cli_plan_->async_send_request(request, std::bind(&Behavior::callbackCliReceivePlan_, this, std::placeholders::_1)).future;
    }

    void Behavior::callbackCliReceivePlan_(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
    {
        (void)future;
        plan_requested_ = false;
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Behavior);
