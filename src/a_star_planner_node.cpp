
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//tf
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h> 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include "a_star_planner/AStar.hpp"


using namespace std;

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);

        ros::NodeHandle nh_;
        
    private:
        ros::Subscriber subOccupancyGrid;
        // ros::Subscriber subLocalPath;
        ros::Subscriber subGoalPose;

        ros::Publisher pubAstarPath; 
        ros::Publisher pubCollisionPoints;

        nav_msgs::Path m_LocalPath;
        geometry_msgs::PoseStamped m_GoalPose;
        bool bNewGoalPose;
};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    subOccupancyGrid = nh_.subscribe("/map",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subGoalPose = nh_.subscribe("/move_base_simple/goal",1, &AstarPlanner::CallbackGoalRviz, this);

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/a_star", 1, true); //nav_msgs/Path.msg
    pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}

void AstarPlanner::CallbackGoalRviz(const geometry_msgs::PoseStamped& msg)
{
    m_GoalPose = msg;
    bNewGoalPose = true;
}

void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
    if(!bNewGoalPose)
        return;

    double target_x = m_GoalPose.pose.position.x; //longitudinal
    double target_y = m_GoalPose.pose.position.y; //lateral

    int row, col, rotate_row, rotate_col;
    int grid_x_size = msg.info.width;
    int grid_y_size = msg.info.height;

    fflush(stdout);
    std::cout <<"grid size: "<< grid_x_size << "," << grid_y_size << "\n";
    fflush(stdout);
    std::cout<<"target: " << target_x << "," << target_y << "\n";


    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({grid_x_size, grid_y_size});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    generator.clearCollisions();

    pcl::PointCloud<pcl::PointXYZI>::Ptr collision_point_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //#1
    // 1. set a origin and target in the grid map.
    // 2. Add collision using generator.addCollision()
    // 3. Visualize a collision points using "collisionCloudMsg"
    
    AStar::Vec2i source = {0, 0};
    AStar::Vec2i target = {target_x, target_y};
    
    for (unsigned int width = 0; width < grid_x_size; width++) {
        for (unsigned int height = 0; height < grid_y_size; height++) {
            //Convert occupied grid to the x-y coordinates to put the target(pcl::PointCloud<pcl::PointXYZI>)
            if (msg.data[height * msg.info.width + width] > 50) {
                pcl::PointXYZI obstacle;
                //geometry_msgs::Pose obstacle;
                obstacle.x = width * msg.info.resolution + msg.info.resolution / 2 + msg.info.origin.position.x;
                obstacle.y = height * msg.info.resolution + msg.info.resolution / 2 + msg.info.origin.position.y;
                obstacle.z = 0;
                obstacle.intensity = msg.data[height * msg.info.width + width];
                collision_point_ptr->push_back(obstacle);
                AStar::Vec2i obstacle_cor = {obstacle.x, obstacle.y};
                generator.addCollision( obstacle_cor ,1 );
                fflush(stdout);
                //cout<<obstacle.x<<","<<obstacle.y<<endl;
            }
        }
    }
    
    //visualize the collision points
    sensor_msgs::PointCloud2 collisionCloudMsg;
    pcl::toROSMsg(*collision_point_ptr, collisionCloudMsg);
    collisionCloudMsg.header.frame_id = "base_link";
    collisionCloudMsg.header.stamp = ros::Time::now();
    pubCollisionPoints.publish(collisionCloudMsg);

    
    // #2
    // 1. Implement A* using generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y})
    
    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time();
    AStartPathMsg.header.frame_id = "base_link";

    int x_shift=0; int y_shift=0;

    if(target.x<0){
        source.x=-target.x+source.x;
        x_shift = target.x;
        target.x = 0;
    }
    if(target.y<0){
        source.y=-target.y+source.y;
        y_shift = target.y;
        target.y = 0;
    }

    
    auto path = generator.findPath(source, target);
    //fflush(stdout);
    //cout<<"path size: "<<path.size()<<endl;
    //ROS_INFO(target_x );

    //#3
    // 1. Publish the result using nav_msgs/Path.msg

    geometry_msgs::PoseStamped _pose;
    _pose.header.seq=0;
    for( auto& coordinate : path ){
         _pose.header.stamp = ros::Time::now();
        _pose.header.frame_id = "base_link";
        _pose.pose.position.x = coordinate.x + x_shift;
        _pose.pose.position.y = coordinate.y + y_shift;
        _pose.pose.position.z = 0;
        AStartPathMsg.poses.push_back(_pose);
        _pose.header.seq++;

        //cout<<path.at(i).x<<","<<path.at(i).y<<endl;
    }
    pubAstarPath.publish(AStartPathMsg);

}

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "a_star_planner");
    // for subscribe
    ros::NodeHandle nh;
    AstarPlanner planner(nh);

    ros::spin();
    return 0;

}
