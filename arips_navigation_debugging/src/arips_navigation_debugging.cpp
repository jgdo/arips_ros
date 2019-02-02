#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <navfn/navfn_ros.h>


int main ( int argc, char **argv )
{
    // Set up ROS.
    ros::init ( argc, argv, "talker" );
    ros::NodeHandle n;
    tf::TransformListener tf ( ros::Duration ( 10 ) );
    costmap_2d::Costmap2DROS local_costmap ( "local_costmap", tf );
    costmap_2d::Costmap2DROS global_costmap ( "global_costmap", tf );

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize ("DWAPlannerROS", &tf, &local_costmap );
    
    navfn::NavfnROS navfn;
    navfn.initialize("my_navfn_planner", &global_costmap);
    
    local_costmap.start();
    global_costmap.start();
    
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    std::vector<geometry_msgs::PoseStamped> global_plan;
    bool newPlan = false;
    
    boost::function<void (const geometry_msgs::PoseStamped&)> callback = 
    [&](const geometry_msgs::PoseStamped& goal) {
        global_plan.clear();
        
        geometry_msgs::PoseStamped start;
        tf::Stamped<tf::Pose> global_pose;
        if(!global_costmap.getRobotPose(global_pose)){
            ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
            return;
        }
        tf::poseStampedTFToMsg(global_pose, start);
    
        //first try to make a plan to the exact desired goal
        
        if(!navfn.makePlan(start, goal, global_plan) || global_plan.empty()){
            ROS_ERROR("Could not plan path");
            return;
        }
        
        newPlan = true;
    };
    
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, callback);

    ros::Rate r ( 2 );

    while ( n.ok() ) {
        if(newPlan) {
            newPlan = false;
            
            if(!dp.setPlan(global_plan)){
                //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                
                global_plan.clear();
            }
        }
        
        if(!global_plan.empty()) {
            geometry_msgs::Twist cmd_vel;
            
            if(dp.computeVelocityCommands(cmd_vel)) {
                vel_pub.publish(cmd_vel);
            } else {
                ROS_INFO("local planner no path");
            }
        }
        
        ros::spinOnce();
        r.sleep();
    }
}


