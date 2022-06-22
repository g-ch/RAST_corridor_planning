#ifndef FSTO_H
#define FSTO_H

#include <decomp_ros_msgs/DynPolyhedronArray.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "CorridorMiniSnap/corridor_minisnap.h"
#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include "fsto/config.h"
#include "fsto/root_finder.hpp"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "sfc_gen.hpp"

using namespace traj_opt;

class Visualization {
 public:
  Visualization(Config &conf, ros::NodeHandle &nh_);

  Config config;
  ros::NodeHandle nh;

  ros::Publisher routePub;
  ros::Publisher wayPointsPub;
  ros::Publisher appliedTrajectoryPub;
  ros::Publisher hPolyPub;
  ros::Publisher textPub;

  void visualize(const Trajectory &appliedTraj,
                 const std::vector<Eigen::Vector3d> &route, ros::Time timeStamp,
                 double compT, double maxV, double totalT);
  void visualizePolyH(const vec_E<Polyhedron3D> &polyhedra,
                      ros::Time timeStamp);
};

class MavGlobalPlanner {
 public:
  MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_);

  Config config;
  ros::NodeHandle nh;

  ros::Subscriber targetSub;
  void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
  ros::Publisher trajPub;
  ros::Publisher corridorPub;

  void publishCorridors(const vec_E<Polyhedron3D> &polyhedra,
                        const Eigen::Matrix3d &initial,
                        const Eigen::Matrix3d &final);

  static void polynomialTrajConverter(
      const Trajectory &traj, quadrotor_msgs::PolynomialTrajectory &trajMsg,
      Eigen::Isometry3d tfR2L, ros::Time &iniStamp);

  EllipsoidDecomp3D cvxDecomp;
  Visualization visualization;
};
#endif