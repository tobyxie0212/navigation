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
#define G 9.81

namespace base_local_planner {

// some very bad global coding -> to be sorted out ======================================
float eo_route_length;
int eo_iter; 
float eo_ocoord[3];

void eoposeToXYTh(geometry_msgs::PoseStamped pose, float coordsarr[]){
  coordsarr[0] = pose.pose.position.x;
  coordsarr[1] = pose.pose.position.y;
  coordsarr[2] = tf::getYaw(pose.pose.orientation);
}

void eocalcl(geometry_msgs::PoseStamped p) {
  float coord[3];
  eoposeToXYTh(p, coord);

  if (eo_iter>0) {
    eo_route_length += hypot(fabs(eo_ocoord[0] - coord[0]), fabs(eo_ocoord[1] - coord[1]));
  }
  eo_iter++;

  for (int j = 0; j < DOF; j++) {
    eo_ocoord[j] = coord[j];
  }
}
// ======================================================================================

EOEnergyCostFunction::EOEnergyCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
  new_set_ = false;
  ROS_INFO(">>> EOEnergyCostFunction Created");

  theta_[0] = 1;
  theta_[1] = .3;
  theta_[2] = .4;
  theta_[3] = .4;
  theta_[4] = .001;
  theta_[5] = .001;
  theta_[6] = .003;
}

EOEnergyCostFunction::~EOEnergyCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}

void EOEnergyCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

bool EOEnergyCostFunction::prepare() {
  new_set_ = true;
  return true;
}

double EOEnergyCostFunction::scoreTrajectory(Trajectory &traj) {
	//
	double test_vx = traj.DWAthetaa_;
	//  
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

  //new added double variables for eo_energy_cost_function
	//ignore [0] for easy name assignment
	// _end suffix represents utility in the rest of the route
	double wheel_rot_vel [5]; 
	double wheel_rot_vel_end [5]; 
	double wheel_vel_p [5]; //wheel velocity in parallel direction
	double wheel_vel_p_end [5];
	double u_static_fric [5]; //static friction of each wheel
	double u_static_fric_end [5];
	double I_motor_pred [5]; //predicted current of each motor for the planned trajectories
	double I_motor_pred_end [5];
	double P_traj_kine, P_traj_fric, P_traj_elec, P_traj_mech, P_traj_idle;
	double P_traj_fric_end, P_traj_elec_end, P_traj_mech_end; //kinetic energy is trivial as assuming none acceleration in the rest of the route
	
	//params for Auckbot TODO:move it to setParams void
	double m_auckbot = 94; //94 kg
	double I_auckbot = 25; //moment of inertia of the robot in Z direction
	
	//TODO:Wednesday remain the default friction coefficients for primitive motions
	double u_rolling_fric = 0.18;
	double u_sliding_fric = 0.9;
	double u_viscous_fric = 0.2 ;					//viscous friction coefficient for robot translation
	double u_viscous_fric_rotation = 0.6; //viscous friction coefficient for robot rotation
	double R_armature = 0.81;
	double M_torque_fric = 0.0195;

  //clear the mean vel&acc in all DOF
  for (int j = 0; j < DOF; j++) {
    vel_mean[j] = 0; 
    acc_mean[j] = 0; 
  }
	



if (n > 1) {
	
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

} else {
	vel_mean[0] = traj.xv_ - traj.DWAxa_*0.025;
	vel_mean[1] = traj.yv_ - traj.DWAya_*0.025;
	vel_mean[2] = traj.thetav_ - traj.DWAthetaa_*0.025;
	
	acc_mean[0] = traj.DWAxa_;
	acc_mean[1] = traj.DWAya_;
	acc_mean[2] = traj.DWAthetaa_;

	traj_length = hypot( vel_mean[0]* 0.05, vel_mean[1]* 0.05);
	
	traj.getPoint(0 , x, y, th);
	rot = fmod(th, (2*PI));

	vel_end[0] = traj.xv_;
	vel_end[1] = traj.yv_;
	vel_end[2] = traj.thetav_;
}
 
  // TRAJECTORY COST   

	//distinguish motion type
	if ( ( (vel_mean[0]==0)&&(vel_mean[1]==0) )||( (vel_mean[1]==0)&&(vel_mean[2]==0) )||( (vel_mean[0]==0)&&(vel_mean[2]==0) ) ) {
	//primitive motion
	u_rolling_fric = 0.18;
	u_sliding_fric = 0.9;
	u_viscous_fric = 0.2;
	u_viscous_fric_rotation = 0.6;
	} else if (vel_mean[0]==0) {
	//YZ strafing curve
	u_rolling_fric = 0.1;
	u_sliding_fric = 0.3;
	u_viscous_fric = 3;
	u_viscous_fric_rotation = 3;
	} else if (vel_mean[1]==0) {
	//XZ forward curve
	u_rolling_fric = 0.1;
	u_sliding_fric = 0.15;
	u_viscous_fric = 0.75;
	u_viscous_fric_rotation = 0.75;
	} else if (vel_mean[2]==0) {
	//XY diagonal
	u_rolling_fric = 0.2;
	u_sliding_fric = 0.25;
	u_viscous_fric = 1;
	u_viscous_fric_rotation = 0;
	} else {
	//XYZ diagonal curve
	u_rolling_fric = 0.1;
	u_sliding_fric = 0.3;
	u_viscous_fric = 1.5;
	u_viscous_fric_rotation = 1.5;
	}

	//joint space: wheel rotational velocity (in pi) (PhD/second year/ros 2d navigation stack/kinematics_matrix_variables)
	wheel_rot_vel[1] = 9.1*( cos(rot)*vel_mean[0] + sin(rot)*vel_mean[1] ) + 9.1*( -sin(rot)*vel_mean[0] + cos(rot)*vel_mean[1] ) + 6 * vel_mean[2];
	wheel_rot_vel[2] = 9.1*( cos(rot)*vel_mean[0] + sin(rot)*vel_mean[1] ) - 9.1*( -sin(rot)*vel_mean[0] + cos(rot)*vel_mean[1] ) + 6 * vel_mean[2];
	wheel_rot_vel[3] = 9.1*( cos(rot)*vel_mean[0] + sin(rot)*vel_mean[1] ) + 9.1*( -sin(rot)*vel_mean[0] + cos(rot)*vel_mean[1] ) - 6 * vel_mean[2];
	wheel_rot_vel[4] = 9.1*( cos(rot)*vel_mean[0] + sin(rot)*vel_mean[1] ) - 9.1*( -sin(rot)*vel_mean[0] + cos(rot)*vel_mean[1] ) - 6 * vel_mean[2];
	
	//kinetic energy - robot motion
	//wheel kinetic energy is too small and neglected
	P_traj_kine = ( fmax(vel_mean[0] * acc_mean[0], 0) + fmax(vel_mean[1] * acc_mean[1], 0) ) * m_auckbot + fmax(vel_mean[2] * acc_mean[2], 0) * I_auckbot;

	//friction dissipation - systain the robot's motion
	if (vel_mean[1]==0 && vel_mean[2]==0) {
	wheel_vel_p[1] = vel_mean[0];
	wheel_vel_p[2] = vel_mean[0];
	wheel_vel_p[3] = vel_mean[0];
	wheel_vel_p[4] = vel_mean[0];
	} else {
	wheel_vel_p[1] = ( vel_mean[0]-vel_mean[2]*(-0.328*cos(rot)+0.328*sin(rot)) )*cos(0.25*PI+rot) + ( vel_mean[1]-vel_mean[2]*(-0.328*sin(rot)-0.328*cos(rot)) )*sin(0.25*PI+rot);
	wheel_vel_p[2] = ( vel_mean[0]-vel_mean[2]*( 0.328*cos(rot)+0.328*sin(rot)) )*cos(0.75*PI+rot) + ( vel_mean[1]-vel_mean[2]*( 0.328*sin(rot)-0.328*cos(rot)) )*sin(0.75*PI+rot);
	wheel_vel_p[3] = ( vel_mean[0]-vel_mean[2]*( 0.328*cos(rot)-0.328*sin(rot)) )*cos(0.25*PI+rot) + ( vel_mean[1]-vel_mean[2]*( 0.328*sin(rot)+0.328*cos(rot)) )*sin(0.25*PI+rot);
	wheel_vel_p[4] = ( vel_mean[0]-vel_mean[2]*(-0.328*cos(rot)-0.328*sin(rot)) )*cos(0.75*PI+rot) + ( vel_mean[1]-vel_mean[2]*(-0.328*sin(rot)+0.328*cos(rot)) )*sin(0.75*PI+rot);
	}

	//determine to use either rolling friction coefficient or sliding friction coefficient (so complex motion determination must be done before here)
	if (wheel_vel_p[1] * wheel_rot_vel[1] >= 0) {
	u_static_fric[1] = u_rolling_fric;
	} else {
	u_static_fric[1] = u_sliding_fric;
	}

	if (wheel_vel_p[2] * wheel_rot_vel[2] >= 0) {
	u_static_fric[2] = u_rolling_fric;
	} else {
	u_static_fric[2] = u_sliding_fric;
	}

	if (wheel_vel_p[3] * wheel_rot_vel[3] >= 0) {
	u_static_fric[3] = u_rolling_fric;
	} else {
	u_static_fric[3] = u_sliding_fric;
	}

	if (wheel_vel_p[4] * wheel_rot_vel[4] >= 0) {
	u_static_fric[4] = u_rolling_fric;
	} else {
	u_static_fric[4] = u_sliding_fric;
	}

	//friction dissipation
	P_traj_fric = u_static_fric[1]*0.25*m_auckbot*G*fabs(wheel_vel_p[1]) + u_static_fric[2]*0.25*m_auckbot*G*fabs(wheel_vel_p[2]) + u_static_fric[3]*0.25*m_auckbot*G*fabs(wheel_vel_p[3]) + u_static_fric[4]*0.25*m_auckbot*G*fabs(wheel_vel_p[4]) + u_viscous_fric*m_auckbot*G*( fabs(vel_mean[0])*fabs(vel_mean[0]) + fabs(vel_mean[1])*fabs(vel_mean[1]) ) + u_viscous_fric_rotation*m_auckbot*G*fabs(vel_mean[2])*fabs(vel_mean[2]);

	//electric dissipation
	I_motor_pred[1] = ( ( cos(rot)-sin(rot))*0.3535*copysign(1.0,wheel_rot_vel[1])*m_auckbot*acc_mean[0] + ( sin(rot)+cos(rot))*0.3535*copysign(1.0,wheel_rot_vel[1])*m_auckbot*acc_mean[1] + 0.54*copysign(1.0,wheel_rot_vel[1])*I_auckbot*acc_mean[2] +u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p[1] + u_static_fric[1]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p[1]) )/36.25;
	I_motor_pred[2] = ( (-cos(rot)-sin(rot))*0.3535*copysign(1.0,wheel_rot_vel[2])*m_auckbot*acc_mean[0] + (-sin(rot)+cos(rot))*0.3535*copysign(1.0,wheel_rot_vel[2])*m_auckbot*acc_mean[1] + 0.54*copysign(1.0,wheel_rot_vel[2])*I_auckbot*acc_mean[2] +u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p[2] + u_static_fric[2]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p[2]) )/36.25;
	I_motor_pred[3] = ( ( cos(rot)-sin(rot))*0.3535*copysign(1.0,wheel_rot_vel[3])*m_auckbot*acc_mean[0] + ( sin(rot)+cos(rot))*0.3535*copysign(1.0,wheel_rot_vel[3])*m_auckbot*acc_mean[1] - 0.54*copysign(1.0,wheel_rot_vel[3])*I_auckbot*acc_mean[2] +u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p[3] + u_static_fric[3]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p[3]) )/36.25;
	I_motor_pred[4] = ( (-cos(rot)-sin(rot))*0.3535*copysign(1.0,wheel_rot_vel[4])*m_auckbot*acc_mean[0] + (-sin(rot)+cos(rot))*0.3535*copysign(1.0,wheel_rot_vel[4])*m_auckbot*acc_mean[1] - 0.54*copysign(1.0,wheel_rot_vel[4])*I_auckbot*acc_mean[2] +u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p[4] + u_static_fric[4]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p[4]) )/36.25;

	if ( ( (vel_mean[0]==0)&&(vel_mean[1]==0) )||( (vel_mean[1]==0)&&(vel_mean[2]==0) )||( (vel_mean[0]==0)&&(vel_mean[2]==0) ) ) {
	//primitive motion model requires current term to be scaled down by four
	I_motor_pred[1] = I_motor_pred[1]/4;
	I_motor_pred[2] = I_motor_pred[2]/4;
	I_motor_pred[3] = I_motor_pred[3]/4;
	I_motor_pred[4] = I_motor_pred[4]/4;
	} 

	P_traj_elec = R_armature * I_motor_pred[1] * I_motor_pred[1] + R_armature * I_motor_pred[2] * I_motor_pred[2] + R_armature * I_motor_pred[3] * I_motor_pred[3] + R_armature * I_motor_pred[4] * I_motor_pred[4];

	//mechanical dissipation
	P_traj_mech = M_torque_fric * wheel_rot_vel[1] * wheel_rot_vel[1] + M_torque_fric * wheel_rot_vel[2] * wheel_rot_vel[2] + M_torque_fric * wheel_rot_vel[3] * wheel_rot_vel[3] + M_torque_fric * wheel_rot_vel[4] * wheel_rot_vel[4];

	//idle consumption - onboard devices consumption
	P_traj_idle = 76.32;	//standstill current 1.59 A

	E_traj = (P_traj_kine + P_traj_fric + P_traj_elec + P_traj_mech + P_traj_idle) * n * t;

  // ROUTE COST
  if (eo_route_length > traj_length) {
    t_route = eo_route_length - traj_length / hypot(vel_end[0], vel_end[1]);
		
		//distinguish motion type
		if ( ( (vel_end[0]==0)&&(vel_end[1]==0) )||( (vel_end[1]==0)&&(vel_end[2]==0) )||( (vel_end[0]==0)&&(vel_end[2]==0) ) ) {
		//primitive motion
		u_rolling_fric = 0.18;
		u_sliding_fric = 0.9;
		u_viscous_fric = 0.2;
		u_viscous_fric_rotation = 0.6;
		} else if (vel_end[0]==0) {
		//YZ strafing curve
		u_rolling_fric = 0.1;
		u_sliding_fric = 0.3;
		u_viscous_fric = 3;
		u_viscous_fric_rotation = 3;
		} else if (vel_end[1]==0) {
		//XZ forward curve
		u_rolling_fric = 0.1;
		u_sliding_fric = 0.15;
		u_viscous_fric = 0.75;
		u_viscous_fric_rotation = 0.75;
		} else if (vel_end[2]==0) {
		//XY diagonal
		u_rolling_fric = 0.2;
		u_sliding_fric = 0.25;
		u_viscous_fric = 1;
		u_viscous_fric_rotation = 0;
		} else {
		//XYZ diagonal curve
		u_rolling_fric = 0.1;
		u_sliding_fric = 0.3;
		u_viscous_fric = 1.5;
		u_viscous_fric_rotation = 1.5;
		}

		//joint space:end wheel rotational velocity (in pi)
		wheel_rot_vel_end[1] = 9.1*( cos(rot)*vel_end[0] + sin(rot)*vel_end[1] ) + 9.1*( -sin(rot)*vel_end[0] + cos(rot)*vel_end[1] ) + 6 * vel_end[2];
		wheel_rot_vel_end[2] = 9.1*( cos(rot)*vel_end[0] + sin(rot)*vel_end[1] ) - 9.1*( -sin(rot)*vel_end[0] + cos(rot)*vel_end[1] ) + 6 * vel_end[2];
		wheel_rot_vel_end[3] = 9.1*( cos(rot)*vel_end[0] + sin(rot)*vel_end[1] ) + 9.1*( -sin(rot)*vel_end[0] + cos(rot)*vel_end[1] ) - 6 * vel_end[2];
		wheel_rot_vel_end[4] = 9.1*( cos(rot)*vel_end[0] + sin(rot)*vel_end[1] ) - 9.1*( -sin(rot)*vel_end[0] + cos(rot)*vel_end[1] ) - 6 * vel_end[2];

		if (vel_end[1]==0 && vel_end[2]==0) {
		wheel_vel_p_end[1] = vel_end[0];
		wheel_vel_p_end[2] = vel_end[0];
		wheel_vel_p_end[3] = vel_end[0];
		wheel_vel_p_end[4] = vel_end[0];
		} else {
		wheel_vel_p_end[1] = ( vel_end[0]-vel_end[2]*(-0.328*cos(rot)+0.328*sin(rot)) )*cos(0.25*PI+rot) + ( vel_end[1]-vel_end[2]*(-0.328*sin(rot)-0.328*cos(rot)) )*sin(0.25*PI+rot);
		wheel_vel_p_end[2] = ( vel_end[0]-vel_end[2]*( 0.328*cos(rot)+0.328*sin(rot)) )*cos(0.75*PI+rot) + ( vel_end[1]-vel_end[2]*( 0.328*sin(rot)-0.328*cos(rot)) )*sin(0.75*PI+rot);
		wheel_vel_p_end[3] = ( vel_end[0]-vel_end[2]*( 0.328*cos(rot)-0.328*sin(rot)) )*cos(0.25*PI+rot) + ( vel_end[1]-vel_end[2]*( 0.328*sin(rot)+0.328*cos(rot)) )*sin(0.25*PI+rot);
		wheel_vel_p_end[4] = ( vel_end[0]-vel_end[2]*(-0.328*cos(rot)-0.328*sin(rot)) )*cos(0.75*PI+rot) + ( vel_end[1]-vel_end[2]*(-0.328*sin(rot)+0.328*cos(rot)) )*sin(0.75*PI+rot);
		}

		if (wheel_vel_p_end[1] * wheel_rot_vel_end[1] >= 0) {
		u_static_fric_end[1] = u_rolling_fric;
		} else {
		u_static_fric_end[1] = u_sliding_fric;
		}

		if (wheel_vel_p_end[2] * wheel_rot_vel_end[2] >= 0) {
		u_static_fric_end[2] = u_rolling_fric;
		} else {
		u_static_fric_end[2] = u_sliding_fric;
		}

		if (wheel_vel_p_end[3] * wheel_rot_vel_end[3] >= 0) {
		u_static_fric_end[3] = u_rolling_fric;
		} else {
		u_static_fric_end[3] = u_sliding_fric;
		}

		if (wheel_vel_p_end[4] * wheel_rot_vel_end[4] >= 0) {
		u_static_fric_end[4] = u_rolling_fric;
		} else {
		u_static_fric_end[4] = u_sliding_fric;
		}

		P_traj_fric_end = u_static_fric_end[1]*0.25*m_auckbot*G*fabs(wheel_vel_p_end[1]) + u_static_fric_end[2]*0.25*m_auckbot*G*fabs(wheel_vel_p_end[2]) + u_static_fric_end[3]*0.25*m_auckbot*G*fabs(wheel_vel_p_end[3]) + u_static_fric_end[4]*0.25*m_auckbot*G*fabs(wheel_vel_p_end[4]) + u_viscous_fric*m_auckbot*G*( fabs(vel_end[0])*fabs(vel_end[0]) + fabs(vel_end[1])*fabs(vel_end[1]) ) + u_viscous_fric_rotation*m_auckbot*G*fabs(vel_end[2])*fabs(vel_end[2]);

		I_motor_pred_end[1] = ( u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p_end[1] + u_static_fric_end[1]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p_end[1]) )/36.25;
		I_motor_pred_end[2] = ( u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p_end[2] + u_static_fric_end[2]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p_end[2]) )/36.25;
		I_motor_pred_end[3] = ( u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p_end[3] + u_static_fric_end[3]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p_end[3]) )/36.25;
		I_motor_pred_end[4] = ( u_viscous_fric*0.25*m_auckbot*G*wheel_vel_p_end[4] + u_static_fric_end[4]*0.25*m_auckbot*G*copysign(1.0,wheel_vel_p_end[4]) )/36.25;

		if ( ( (vel_end[0]==0)&&(vel_end[1]==0) )||( (vel_end[1]==0)&&(vel_end[2]==0) )||( (vel_end[0]==0)&&(vel_end[2]==0) ) ) {
		//primitive motion model requires current term scale down
		I_motor_pred_end[1] = I_motor_pred_end[1]/4;
		I_motor_pred_end[2] = I_motor_pred_end[2]/4;
		I_motor_pred_end[3] = I_motor_pred_end[3]/4;
		I_motor_pred_end[4] = I_motor_pred_end[4]/4;
		} 

		P_traj_elec_end = R_armature * I_motor_pred_end[1] * I_motor_pred_end[1] + R_armature * I_motor_pred_end[2] * I_motor_pred_end[2] + R_armature * I_motor_pred_end[3] * I_motor_pred_end[3] + R_armature * I_motor_pred_end[4] * I_motor_pred_end[4];

		P_traj_mech_end = M_torque_fric * wheel_rot_vel_end[1] * wheel_rot_vel_end[1] + M_torque_fric * wheel_rot_vel_end[2] * wheel_rot_vel_end[2] + M_torque_fric * wheel_rot_vel_end[3] * wheel_rot_vel_end[3] + M_torque_fric * wheel_rot_vel_end[4] * wheel_rot_vel_end[4];
		
    E_route = (P_traj_fric_end + P_traj_elec_end + P_traj_mech_end + P_traj_idle)*t_route;

    traj_scale = traj_length / eo_route_length;       
  } else { // trajectory leads already to goal
    E_route = 0;
    traj_scale = 1; 
  }

	//traj_scale += 0.3;
	if (traj_scale > 1) traj_scale = 1;
	//TODO: self_scale used to scale the energy term in the DWA cost functiion
	self_scale = 0.7;

  // ROS_INFO(">>> scoreTrajectory s:%d, l:%.2f, r:%.2f, v:%.3f, a:%.3f, e:%.3f", \
  //   traj.getPointsSize(), traj_length, rot, vel[0][0], acc[0][0], E_traj);
  // ROS_INFO("-- lengths: %.1f // %.1f", traj_length, route_length);
  // ROS_INFO("HYPO TEST (all shoud be positive) %.1f, %.1f, %.1f", hypot(-3.0, -4.0), hypot(3.0, -4.0), hypot(-3.0, 4.0));

  cost = E_traj * traj_scale + E_route * (1-traj_scale) ;
	cost *= self_scale;

	ROS_INFO(">>> traj_scale: %4.2f, cost: %3.1f", traj_scale, cost);
	ROS_INFO("    traj_length: %5.2f, eo_route_length: %5.2f", traj_length, eo_route_length);
  return cost;
}

void EOEnergyCostFunction::setLastSpeeds(double x, double y, double th) {
  last_speeds_[0] = x;
  last_speeds_[1] = y;
  last_speeds_[2] = th;
}

void EOEnergyCostFunction::thetaCallback(const auckbot_analysis::ModelTheta msg) {
  ROS_INFO("------> thetaCallback <----");
}

void EOEnergyCostFunction::setRoute(std::vector<geometry_msgs::PoseStamped> global_plan) {
  route_ = global_plan;
  geometry_msgs::PoseStamped goal = route_.back();

  float goalc[3];
  eoposeToXYTh(goal, goalc);
  // ROS_INFO("Goal: %f, %f, %f", goalc[0], goalc[1], goalc[2]);

  eo_iter = 0;
  eo_route_length = 0;
  for_each (route_.begin(), route_.end(), eocalcl);
  // ROS_INFO("Length: %f", eo_route_length);
}

} /* namespace base_local_planner */
