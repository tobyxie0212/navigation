/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redisribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redisributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redisributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the disribution.
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/eo_energy_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

#include <iostream>
#include <fstream>

#define PI 3.14159265359
#define DOF 3
#define N_MAX 50

namespace base_local_planner {

// some very bad global coding -> to be sorted out ======================================
float route_length;
int iter; 
float ocoord[3];

void poseToXYTh(geometry_msgs::PoseStamped pose, float coordsarr[]){
  coordsarr[0] = pose.pose.position.x;
  coordsarr[1] = pose.pose.position.y;
  coordsarr[2] = tf::getYaw(pose.pose.orientation);
}

void calcl(geometry_msgs::PoseStamped p) {
  float coord[3];
  poseToXYTh(p, coord);

  if (iter>0) {
    route_length += hypot(fabs(ocoord[0] - coord[0]), fabs(ocoord[1] - coord[1]));
  }
  iter++;

  for (int j = 0; j < DOF; j++) {
    ocoord[j] = coord[j];
  }
}
// ======================================================================================

EnergyCostFunction::EnergyCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
  new_set_ = false;
  ROS_INFO(">>> EnergyCostFunction Created");

  theta_[0] = 1;
  theta_[1] = .3;
  theta_[2] = .4;
  theta_[3] = .4;
  theta_[4] = .001;
  theta_[5] = .001;
  theta_[6] = .003;
}

EnergyCostFunction::~EnergyCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}

void EnergyCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

bool EnergyCostFunction::prepare() {
  new_set_ = true;
  return true;
}

double EnergyCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double x, y, th;
  double ox, oy, oth;
  double traj_length = 0;
  double rot = 0;
  double n = traj.getPointsSize();
  double dis [3][N_MAX];	//
  double vel [3][N_MAX];  //
  double vel_mean [3];
  double vel_end [3];
  double acc [3][N_MAX];
  double acc_mean [3];
  double t = traj.time_delta_;
  double E_traj, E_route, E_total;
  double t_route = 0;
  double traj_scale = 0;
  double self_scale = 0;

  //clear the mean vel&acc in all DOF
  for (int j = 0; j < DOF; j++) {
    vel_mean[j] = 0; 
    acc_mean[j] = 0; 
  }

  int i; // (WE WILL NEED THE i LATER !!)
  for (i = 0; i < n; ++i) { 
    traj.getPoint(i, x, y, th);
    dis [0][i] = x;
    dis [1][i] = y;
    dis [2][i] = th;
    
    if (i > 0) // velocity
    {
      // x
      vel[0][i-1] = ( sin(dis[2][i-1])*(dis[0][i-1] - dis[0][i]) + cos(dis[2][i-1])*(dis[1][i-1] - dis[1][i]) ) / t;
      // y
      vel[1][i-1] = ( cos(dis[2][i-1])*(dis[0][i-1] - dis[0][i]) - sin(dis[2][i-1])*(dis[1][i-1] - dis[1][i]) ) / t;
      // theta
      vel[2][i-1] = fmod(fabs(dis[2][i-1] - dis[2][i]), (2*PI)) / t;

      // mean
      for (int j = 0; j < DOF; j++) {
        vel_mean[j] += fabs(vel[j][i-1]);
      }

      // traj_length~s
      traj_length += hypot((dis[0][i-1] - dis[0][i]), (dis[1][i-1] - dis[1][i]));
      rot += fmod(fabs(dis[2][i-1] - dis[2][i]), (2*PI));
    }
    if (i > 1) // acceleration
    {
      // x
      acc[0][i-2] = ( sin(dis[2][i-2])*(vel[0][i-2] - vel[0][i-1]) + cos(dis[2][i-2])*(vel[1][i-2] - vel[1][i-1]) ) / t;
      // y
      acc[1][i-2] = ( cos(dis[2][i-2])*(vel[0][i-2] - vel[0][i-1]) - sin(dis[2][i-2])*(vel[1][i-2] - vel[1][i-1]) ) / t;
      // theta
      acc[2][i-2] = fmod(fabs(vel[2][i-2] - vel[2][i-1]), (2*PI)) / t;

      // mean
      for (int j = 0; j < DOF; j++) {
        acc_mean[j] += acc[j][i-2];
      }
    }
  }
  // end 
  for (int j = 0; j < DOF; j++) {
    vel_end[j] += vel[j][i-1]; // (WE STILL USE THE i !!)
  }

  for (int j = 0; j < DOF; j++) {
    vel_mean[j] /= n-1;
    acc_mean[j] /= n-2; 
  }
 
  // TRAJECTORY COST   
  E_traj = 
   (theta_[0] +                       //  th_1
    theta_[1] * fabs(vel_mean[0]) +   //  v_x
    theta_[2] * fabs(vel_mean[1]) +   //  v_y
    theta_[3] * fabs(vel_mean[2]) +   //  v_th
    theta_[4] * fabs(acc_mean[0]) +         //  a_x
    theta_[5] * fabs(acc_mean[1]) +         //  a_y
    theta_[6] * fabs(acc_mean[2]) ) * //  a_th
    n * t /                           //  * duration (n:size of each delta traj, t:time of each delta traj
    hypot(traj_length, rot);          //  / distance

  // ROUTE COST
  if (route_length > traj_length) {
    t_route = route_length - traj_length / hypot(vel_end[0], vel_end[1]);
    E_route = 
     (theta_[0] +                       //  th_1
      theta_[1] * fabs(vel_end[0]) +    //  v_x
      theta_[2] * fabs(vel_end[1]) +    //  v_y
      theta_[3] * fabs(vel_end[2]) ) *  //  v_th
      t_route /                         //  * duration
      route_length - traj_length;       //  / distance
    traj_scale = traj_length / route_length;       
  } else { // trajectory leads already to goal
    E_route = 0;
    traj_scale = 1; 
  }

	traj_scale += 0.3;
	if (traj_scale > 1) traj_scale = 1;
	self_scale = 0.7;

  // ROS_INFO(">>> scoreTrajectory s:%d, l:%.2f, r:%.2f, v:%.3f, a:%.3f, e:%.3f", \
  //   traj.getPointsSize(), traj_length, rot, vel[0][0], acc[0][0], E_traj);
  // ROS_INFO("-- lengths: %.1f // %.1f", traj_length, route_length);
  // ROS_INFO("HYPO TEST (all shoud be positive) %.1f, %.1f, %.1f", hypot(-3.0, -4.0), hypot(3.0, -4.0), hypot(-3.0, 4.0));

  cost = E_traj * traj_scale + E_route * (1-traj_scale) ;
	cost *= self_scale;

	ROS_INFO(">>> traj_scale: %4.2f, cost: %3.1f", traj_scale, cost);
	ROS_INFO("    traj_length: %5.2f, route_length: %5.2f", traj_length, route_length);
  return cost;
}

void EnergyCostFunction::setLastSpeeds(double x, double y, double th) {
  last_speeds_[0] = x;
  last_speeds_[1] = y;
  last_speeds_[2] = th;
}

void EnergyCostFunction::thetaCallback(const auckbot_analysis::ModelTheta msg) {
  ROS_INFO("------> thetaCallback <----");
}

void EnergyCostFunction::setRoute(std::vector<geometry_msgs::PoseStamped> global_plan) {
  route_ = global_plan;
  geometry_msgs::PoseStamped goal = route_.back();

  float goalc[3];
  poseToXYTh(goal, goalc);
  // ROS_INFO("Goal: %f, %f, %f", goalc[0], goalc[1], goalc[2]);

  iter = 0;
  route_length = 0;
  for_each (route_.begin(), route_.end(), calcl);
  // ROS_INFO("Length: %f", route_length);
}

} /* namespace base_local_planner */
