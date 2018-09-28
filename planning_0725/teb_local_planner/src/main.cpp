#include <teb_local_planner/teb_local_planner_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <nav_msgs/OccupancyGrid.h>
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
void GlobalPlanCB(const nav_msgs::Path& msg);
void LocalPlanCB(const ros::TimerEvent& e);
//void LocalCostMapCB(const nav_msgs::OccupancyGrid& msg);
void publishZero();
void resetState();
//void costmapCB(const ****& msg);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_local_planner");
    ros::NodeHandle n("~");
    tf_ = new tf::TransformListener(ros::Duration(10));
    config.loadRosParamFromNodeHandle(n);
    ros::Timer planner_timer = n.createTimer(ros::Duration(5),LocalPlanCB);
    
    dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
    dynamic_recfg->setCallback(cb);
    
    vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Subscriber real_goal = n.subscribe("/move_base_simple/goal", 1, GoalCB);
    ros::Subscriber gl_planer = n.subscribe("global_plan",1,GlobalPlanCB);
    
    //ros::Subscriber local_grid = n.subscribe("local_costmap",1,LocalCostMapCB);
    //ros::Subscriber timed_costmap_ = n.subscribe("timed_costmap",1,costmapCB);
    global_plan = new std::vector<geometry_msgs::PoseStamped>();
    local_plan = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan = new std::vector<geometry_msgs::PoseStamped>();
    
    global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", *tf_);
    global_costmap_->pause();
    local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", *tf_);
    local_costmap_->pause();
    teb_local.initialize("local_planner", tf_, local_costmap_);
    global_costmap_->start();
    local_costmap_->start();
    ros::spin();
    return 0;
}

void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped temp = *msg;
    if(temp.pose.position.x != goal.pose.position.x || temp.pose.position.y != goal.pose.position.y){
        goal = temp;
        new_goal = true;
    }
    
}

void GlobalPlanCB(const nav_msgs::Path& msg){
    if(new_goal){
        *global_plan = msg.poses;
        new_global_plan = true;
        new_goal = false;
        trajectory = true;
    }
    
}

void LocalPlanCB(const ros::TimerEvent& e){
    if(!goal_reached){
        geometry_msgs::Twist cmd_vel;
        if(new_global_plan){
            new_global_plan = false;
            local_plan = global_plan;
            if(!teb_local.setPlan(*local_plan)){
                ROS_ERROR("setPlan Error");
                resetState();
                return;
            }
        }
        if(trajectory){
            if(teb_local.isGoalReached()){
                ROS_DEBUG_NAMED("move_base","Goal reached!");
                goal_reached = true;
                trajectory = false;
            }
            if(teb_local.computeVelocityCommands(cmd_vel)){//compute中实现了可视化，publish planner,global_planner_
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
    goal_reached = false;
    trajectory = false;
}
