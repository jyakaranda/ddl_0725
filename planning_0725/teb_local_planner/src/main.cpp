#include <teb_local_planner/teb_local_planner_ros.h>
#include <math.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_converter/costmap_converter_interface.h>
//#include <vector>
#include <geometry_msgs/Twist.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <nav_core/recovery_behavior.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;

geometry_msgs::PoseStamped goal;
//三个vector用来存储plan
std::vector<geometry_msgs::PoseStamped>* global_plan;
std::vector<geometry_msgs::PoseStamped>* latest_plan;
std::vector<geometry_msgs::PoseStamped>* local_plan;
//costmap
costmap_2d::Costmap2DROS* planner_costmap_ros, *controller_costmap_ros;
tf::TransformListener* tf_;
TebLocalPlannerROS teb_local;
//tf::TransformListener tf_;
ros::Publisher vel_pub_;
// =========== Function declarations =============
clear_costmap_recovery::ClearCostmapRecovery ccr;
//nav_core::RecoveryBehavior nrb;
//add
void CB_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);
//void CB_initialization(const geometry_msgs::PoseStamped::ConstPtr& msg);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_global(const nav_msgs::Path& mesg);
//将global_planner转换成local_plan并且发布出去

//void executeCycle(const geometry_msgs::PoseStamped& goal, const std::vector<geometry_msgs::PoseStamped>& plan);
//上面的函数用于进行下面函数的判断，判断是否为True
void executeLocal_plan(const ros::TimerEvent& e);
bool new_global_plan = false;//用来判断global_plan是否更新，从而进行局部路径规划的更新
bool new_goal = false;//用来判断goal是否变化，然后来重新获取global_plan的信息
bool goal_reached = false;
enum MovebaseState{
  PLANNING,
  CONTROLLING,
  CLEARING
};
MovebaseState state_;

// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
  //tf::TransformListener &tf(ros::Duration(10));
  tf_ = new tf::TransformListener(ros::Duration(10));
  //tf::TransformListener& tf_ =  *tf;
 
  ccr = clear_costmap_recovery::ClearCostmapRecovery(); 
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
  ros::Timer plan_timer = n.createTimer(ros::Duration(1),executeLocal_plan);//循环调用，当目标进行更新了之后，重新订阅global的信息，然后
  //ros::Timer goal_timer = n.createTimer(ros::Duration(1),CB_goal);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  //订阅global plan
  ros::Subscriber global_p = n.subscribe("/global_plan",1,CB_global);
  
  //订阅move_base_goal节点
  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal",1,CB_goal);

  //make_plan_srv = n.advertiseService("make_plan",,this)

  state_ = PLANNING;//一开始，我们需要进行路径规划
  //路径初始化
  global_plan = new std::vector<geometry_msgs::PoseStamped>();
  latest_plan = new std::vector<geometry_msgs::PoseStamped>();
  local_plan = new std::vector<geometry_msgs::PoseStamped>();
  planner_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap",*tf_);
  controller_costmap_ros = new costmap_2d::Costmap2DROS("local_costmap",*tf_);
  controller_costmap_ros->pause();
  planner_costmap_ros->pause();

  //之前订阅global_planner 获得了global_planner的信息

  teb_local.initialize("local_planner", tf_, controller_costmap_ros);

  //更新costmap
  planner_costmap_ros->start();
  controller_costmap_ros->start();

  ros::spin();

  return 0;
}


//订阅goal的信息
void CB_goal(const geometry_msgs::PoseStamped::ConstPtr& msg ){
  goal = *msg;
  new_goal = true;
  goal_reached = false;
}

//订阅global_planner的信息
void CB_global(const nav_msgs::Path& mesg){
  if(new_goal){
    *global_plan = mesg.poses;
    std::vector<geometry_msgs::PoseStamped>* temp_plan = global_plan;
    global_plan = latest_plan;
    latest_plan = temp_plan;
    new_global_plan = true;
    new_goal = false;
    state_ = CONTROLLING;
  }
}
/**
void executeCycle(const geometry_msgs::PoseStamped& goal, const std::vector<geometry_msgs::PoseStamped>& plan){
  if(!goal_reached){
    executeLocal_plan(goal, plan);
  }
}
*/
//实现将global_Plan转换成local_plan的函数，参数为goal和global_plan
void executeLocal_plan(const ros::TimerEvent& e){
  if(!goal_reached){
  geometry_msgs::Twist cmd_vel;
  
  if(new_global_plan){
    new_global_plan = false;
    std::vector<geometry_msgs::PoseStamped>* temp_plan;
    local_plan = latest_plan;
    latest_plan = temp_plan;
    if(!teb_local.setPlan(*local_plan)){
      ROS_ERROR("local plan error");
      state_ = PLANNING;
    }
  }
  switch(state_){
    case PLANNING:
      {

      }
      break;
    case CONTROLLING:
      {
        ROS_DEBUG_NAMED("move_base","In controlling state.");
        if(teb_local.isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          state_ = PLANNING;
          goal_reached = true;
        }
        if(teb_local.computeVelocityCommands(cmd_vel)){//compute中实现了可视化，publish planner,global_planner_
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
	  vel_pub_.publish(cmd_vel);
        }else{
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
        }
	break;
      }
      
  }
  }else{
    ROS_INFO("Goal Reached!");
    ccr.initialize("my_recovery_costmap",tf_, planner_costmap_ros, controller_costmap_ros);
    ccr.runBehavior();
  }
}
/**
Eigen::Vector3d<float> comptheta(geometry_msgs::PoseStamped& goal){
  float theta_x,theta_y,theta_z;
  float q0,q1,q2,q3;
  theta_x = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  theta_y = arcsin(2*(q0*q2-q1*q3));
  theta_z = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
  Eigen::Vector3d<float> theta_value(theta_x,theta_y,theta_z);
  return theta_value;
}


*/
void CB_reconfigure(teb_local_planner::TebLocalPlannerReconfigureConfig& reconfig, uint32_t level){
  config.reconfigure(reconfig);
}
