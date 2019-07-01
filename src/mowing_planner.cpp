/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Kosuke Inoue
*********************************************************************/
#include <mowing_planner/mowing_planner.h>
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <unistd.h>
#include <math.h>

geometry_msgs::PoseStamped prev_goal;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped new_goal;
ros::NodeHandle nh("point_pub");
ros::Publisher point_pub = nh.advertise<geometry_msgs::PoseStamped>("point_on_path", 10);

//頂点の定義
struct Vertex2D{
        double x;
        double y;
};
//ベクトルの定義(頂点と同じ)
#define Vector2D Vertex2D

//単位ベクトル生成
Vector2D ceate_unit_vector( Vector2D v )
{
        double len = pow( ( v.x * v.x ) + ( v.y * v.y ), 0.5 );//ベクトル長さ

        Vector2D ret;
        ret.x = v.x / len;
        ret.y = v.y / len;

        return ret;
}

//ベクトル内積
double dot_product(Vector2D vl, Vector2D vr) {
        return vl.x * vr.x + vl.y * vr.y;
}

//点Pと直線ABから線上最近点を求める
Vector2D NearPosOnLine(Vertex2D P, Vertex2D A, Vertex2D B )
{
        Vector2D AB,AP;//ベクトルAB AP

        AB.x = B.x - A.x;
        AB.y = B.y - A.y;
        AP.x = P.x - A.x;
        AP.y = P.y - A.y;

        //ABの単位ベクトルを計算
        Vector2D nAB = ceate_unit_vector(AB);

        //Aから線上最近点までの距離（ABベクトルの後ろにあるときはマイナス値）
        double dist_AX = dot_product( nAB, AP );

        //線上最近点
        Vector2D ret;
        ret.x = A.x + ( nAB.x * dist_AX );
        ret.y = A.y + ( nAB.y * dist_AX );

        return ret;
}


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mowing_planner::MowingPlanner, nav_core::BaseGlobalPlanner)

namespace mowing_planner {

  MowingPlanner::MowingPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  MowingPlanner::MowingPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  
  void MowingPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      
      tf::TransformListener listener;
      tf::StampedTransform transform;
      try{
          listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
          listener.lookupTransform("map", "base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      prev_goal.header.frame_id = "map";
      prev_goal.pose.position.x = transform.getOrigin().x();
      prev_goal.pose.position.y = transform.getOrigin().y();
      prev_goal.pose.position.z = transform.getOrigin().z();
      prev_goal.pose.orientation.x = transform.getRotation().x();
      prev_goal.pose.orientation.y = transform.getRotation().y();
      prev_goal.pose.orientation.z = transform.getRotation().z();
      prev_goal.pose.orientation.w = transform.getRotation().w();
      
      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double MowingPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }

  bool MowingPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
//    ros::Subscriber sub = nh.subscribe("/move_base/current_goal", 100, goal_callback);

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The mowing planner could not find a valid plan for this goal");
        break;
      }
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
      
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 0)
      {
          done = true;
      }
      scale -=dScale;
    }
      
      //if we have no footprint... do nothing
    if(goal.header.stamp != new_goal.header.stamp){
        prev_goal.pose.position.x = new_goal.pose.position.x;
        prev_goal.pose.position.y = new_goal.pose.position.y;
        point_pub.publish(prev_goal);
}
      
      new_goal = goal;
      tf2::Quaternion goal_quat;
      goal_quat.setRPY(0, 0, target_yaw);

      current_pose = start;

      Vertex2D start_vector;
      start_vector.x = prev_goal.pose.position.x;
      start_vector.y = prev_goal.pose.position.y;
      Vertex2D goal_vector;
      goal_vector.x = new_goal.pose.position.x;
      goal_vector.y = new_goal.pose.position.y;
      Vertex2D corrent_position;
      corrent_position.x = current_pose.pose.position.x;
      corrent_position.y = current_pose.pose.position.y;

      new_goal.pose.position.x = target_x;
      new_goal.pose.position.y = target_y;

      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w(); 
    
      Vertex2D point_on_line;
      point_on_line = NearPosOnLine(corrent_position, start_vector, goal_vector);
    int divided_number = 50;
    for(int i=0;i<divided_number;i++){
    geometry_msgs::PoseStamped mid_goal;
    mid_goal.header.frame_id = "map";
    mid_goal.header.stamp = current_pose.header.stamp;
    mid_goal.pose.position.x = point_on_line.x + (new_goal.pose.position.x-point_on_line.x)/divided_number * i;
    mid_goal.pose.position.y = point_on_line.y + (new_goal.pose.position.y-point_on_line.y)/divided_number * i;
    mid_goal.pose.orientation = new_goal.pose.orientation; 
    plan.push_back(mid_goal);
    }
    plan.push_back(new_goal);
    return (done);
}
};
