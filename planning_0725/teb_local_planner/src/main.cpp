#include <teb_local_planner/teb_local_planner_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/time.h>
using namespace teb_local_planner;

TebConfig config;
tf::TransformListener *tf_;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;


std::vector<geometry_msgs::PoseStamped>* global_plan;
std::vector<geometry_msgs::PoseStamped>* local_plan;
std::vector<geometry_msgs::PoseStamped>* latest_plan;

geometry_msgs::PoseStamped goal;//终点信息

TebLocalPlannerROS teb_local;

//订阅一个costmap的信息
costmap_2d::Costmap2DROS* local_costmap_, * global_costmap_;
ros::Publisher vel_pub_;

bool new_goal = false;
bool new_global_plan = false;
bool goal_reached = false;
bool trajectory = false;
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
//void GlobalPlanCB(const geometry_msgs::PolygonStamped& msg);
void GlobalPlanCB(const nav_msgs::Path& msg);
void LocalPlanCB(const ros::TimerEvent& e);
void LocalCostMapCB(const nav_msgs::OccupancyGrid& msg);
void publishZero();
void resetState();
//void costmapCB(const ****& msg);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_local_planner");
    ros::NodeHandle n("~");
    tf_ = new tf::TransformListener(ros::Duration(10));
    config.loadRosParamFromNodeHandle(n);
    ros::Timer planner_timer = n.createTimer(ros::Duration(0.2),LocalPlanCB);
    
    dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
    dynamic_recfg->setCallback(cb);
    
    vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Subscriber real_goal = n.subscribe("/move_base_simple/goal", 0.5, GoalCB);
    ros::Subscriber gl_planer = n.subscribe("/trajectory/current",0.5,GlobalPlanCB);
    
    //ros::Subscriber local_grid = n.subscribe("local_costmap",1,LocalCostMapCB);
    //ros::Subscriber timed_costmap_ = n.subscribe("timed_costmap",1,costmapCB);
    global_plan = new std::vector<geometry_msgs::PoseStamped>();
    local_plan = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan = new std::vector<geometry_msgs::PoseStamped>();
    
    global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", *tf_);
    global_costmap_->pause();
    local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", *tf_);
    local_costmap_->pause();
    teb_local.initialize("TebLocalPlannerROS", tf_, local_costmap_);
    global_costmap_->start();
    local_costmap_->start();
    ros::spin();
    return 0;
}

void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("receive goal!");
    geometry_msgs::PoseStamped temp = *msg;
    if(temp.pose.position.x != goal.pose.position.x || temp.pose.position.y != goal.pose.position.y){
        goal = temp;
       // ROS_INFO("%f %f %f %f", temp.pose.orientation.x, temp.pose.orientation.y,temp.pose.orientation.z, temp.pose.orientation.w);
        new_goal = true;
        global_plan->clear();
        local_plan->clear(); 
    }
    
}
/**
void GlobalPlanCB(const geometry_msgs::PolygonStamped& msg){
    if(new_goal){
        for(int i = 0; i< msg.polygon.points.size(); i++){
            geometry_msgs::PoseStamped message = geometry_msgs::PoseStamped();
            message.pose.position.x = msg.polygon.points[i].x;
            message.pose.position.y = msg.polygon.points[i].y;
            message.pose.position.z = msg.polygon.points[i].z;
            message.pose.orientation.x = 0;
            message.pose.orientation.y = 0;
            message.pose.orientation.z = 0;
            message.pose.orientation.w = 0;
            global_plan->push_back(message);
        }
        new_global_plan = true;
        //new_goal = false;
        trajectory = true;
    }
    
}
*/

void GlobalPlanCB(const nav_msgs::Path& msg){
    if(new_goal){
        ROS_INFO("receive goal, subscriber global planner");
        global_plan->clear();
        geometry_msgs::PoseStamped plan_point; 
        for(int i =0; i<msg.poses.size();i++){
            plan_point.pose = msg.poses[i].pose;
            plan_point.header = msg.header;
            plan_point.header.seq = i+1;
            plan_point.header.frame_id="map";
            global_plan->push_back(plan_point);
            //ROS_INFO("%f %f %f %f",  plan_point.pose.orientation.x, plan_point.pose.orientation.y, plan_point.pose.orientation.z, plan_point.pose.orientation.w);
        }
        new_goal=false;
        new_global_plan = true;
        trajectory = true;
        goal_reached = false;
        /**
        if(!goal_reached){
            geometry_msgs::Twist cmd_vel;
            if(new_global_plan){
                ROS_INFO("new_global_plan");
                new_global_plan = false;
                if(!teb_local.setPlan(*global_plan)){
                    ROS_ERROR("setPlan Error");
                    resetState();
                    return;
                }
            }
            while(trajectory){
                ROS_INFO("trajectory start");
                if(teb_local.isGoalReached()){
                    ROS_DEBUG_NAMED("move_base","Goal reached!");
                    goal_reached = true;
                    //trajectory = false;
                    resetState();
                }
            //ROS_INFO("AAAAAAAAAA");
                if(teb_local.computeVelocityCommands(cmd_vel)){//compute中实现了可视化，publish planner,global_planner_
                //ROS_INFO("BBBB");
                    ROS_INFO("%f", cmd_vel.linear.x);
                    ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                    vel_pub_.publish(cmd_vel);
                }else{
                    ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                }
            //ROS_INFO("BBBBBBBBBBB");
            }
        }
     */
    }
}

void LocalPlanCB(const ros::TimerEvent& e){
    if(!goal_reached){
        geometry_msgs::Twist cmd_vel;
        if(new_global_plan){
            ROS_INFO("new_global_plan");
            new_global_plan = false;
            if(!teb_local.setPlan(*global_plan)){
                ROS_ERROR("setPlan Error");
                resetState();
                return;
            }
        }
        if(trajectory){
            ROS_INFO("trajectory start");
            if(teb_local.isGoalReached()){
                ROS_INFO("Goal reached!");
                goal_reached = true;
                //trajectory = false;
                resetState();
                
            }
            if(teb_local.computeVelocityCommands(cmd_vel)){//compute中实现了可视化，publish planner,global_planne
                ROS_INFO("%f %f %f", cmd_vel.linear.x,cmd_vel.linear.y, cmd_vel.angular.z );

                ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                vel_pub_.publish(cmd_vel);
            }else{
                ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
            }
        }
    }
}
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
    config.reconfigure(reconfig);
}

void publishZero(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    vel_pub_.publish(cmd_vel);
}

void resetState(){
    new_goal = false;
    new_global_plan = false;
    goal_reached = true;
    trajectory = false;
}
