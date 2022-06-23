//
// Created by clarence on 2022/2/8.
//

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <queue>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "risk_aware_kinodynamic_a_star.h"
#include "decomp_ros_msgs/DynPolyhedronArray.h"
#include "decomp_ros_msgs/Polyhedron.h"
#include <CorridorMiniSnap/corridor_minisnap.h>
#include "nav_msgs/Path.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "std_msgs/Float32MultiArray.h"
#include "mav_msgs/default_topics.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <ctime>
#include <chrono>
#include "std_msgs/UInt8.h"


using namespace traj_opt;
using namespace std;
using namespace std::chrono;

Astar astar_planner;

CorridorMiniSnap mini_snap_;
Trajectory traj_;

queue<double> pose_att_time_queue;
queue<Eigen::Vector3d> uav_position_global_queue;
queue<Eigen::Quaternionf> uav_att_global_queue;

ros::Publisher corridor_pub, color_vel_pub, trajectory_pub, pva_pub, pva_traj_pub, velocity_setpoint_pub, position_tracking_pub;
ros::Publisher current_marker_pub, fov_pub, astar_result_pub, mode_pub, tracking_error_too_large_state_pub;

Eigen::Vector3d uav_position_global;
Eigen::Vector3d uav_velocity_global;
Eigen::Vector3d uav_acceleration_global;
Eigen::Quaternionf uav_att_global;

bool state_locked = false;
double max_differentiated_current_a = 4.0;

float future_risk_global[VOXEL_NUM][RISK_MAP_NUMBER];
bool future_risk_updated = false;
bool future_risk_locked = false;

bool rviz_map_center_locked = false;

bool position_received = false;
bool trajectory_initialized = false;
double goal_x = 60.0, goal_y = 0.0, goal_z = 1.5;

bool in_safety_mode = false;
Eigen::Vector3d p_store_for_em, v_store_for_em, a_store_for_em;
float yaw_store_for_em = 0.f;
float reference_direction_angle = 100.f;

typedef struct PVAYPoint
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double yaw = 0.0;
}PVAYPoint;

queue<PVAYPoint> trajectory_piece;
queue<PVAYPoint> trajectory_to_nmpc;

int trajectory_piece_max_size = 12;
int nmpc_receive_points_num = 20;
double planning_time_step = 0.05;

double max_vel = 3.0;
double max_acc = 4.0;
double max_vel_optimization = 3.0;
double max_acc_optimization = 4.0;
double delta_corridor = 0.3;

bool use_height_limit = true;
float height_limit_max = 2.2f;
float height_limit_min = 0.f;
bool sample_z_acc = true;

float a_star_acc_sample_step = 2.f;
float a_star_search_time_step = 0.4f; /// Should be tuned to reach one voxel at least when using maximum acceleration.
float expand_safety_distance = 0.2f;

float risk_threshold_motion_primitive = 0.15;
float risk_threshold_single_voxel = 0.15;
float risk_threshold_corridor = 2.5;

std::vector<double> factors(5);

geometry_msgs::PoseStamped map_pose_global;

bool optimizationInCorridors(const decomp_ros_msgs::DynPolyhedronArray msg, const Eigen::Vector3d planning_start_map_center);

void corridorsPublish(vector<Corridor*> &corridors, geometry_msgs::PoseStamped &map_pose, bool clear_corridors = false)
{
    ROS_INFO("corridors num = %ld", corridors.size());

    if(clear_corridors){
        corridors.clear();
        auto* empty_corridor = new Corridor();
        for(int i=0; i<10; ++i){
            for(int j=0; j<8; ++j)
            {
                Point3D p;
                p.x = p.y = p.z = 1000.f;
                empty_corridor->envelope.vertexes.push_back(p);
            }
            corridors.push_back(empty_corridor);
        }
    }else{
        if(corridors.empty()) {
            ROS_INFO("Empty corridors !");
            return;
        }
    }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "cubes";

    marker.color.a = 0.2;
    marker.color.g = 1.0;

    int id=0;
    for(const auto& c : corridors){
        marker.id = id;
        ++id;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        for(int i=0; i<8; ++i)
        {
            marker.pose.position.x += c->envelope.vertexes[i].x;
            marker.pose.position.y += c->envelope.vertexes[i].y;
            marker.pose.position.z += c->envelope.vertexes[i].z;
        }
        marker.pose.position.x /= 8.0;
        marker.pose.position.y /= 8.0;
        marker.pose.position.z /= 8.0;

        if(!rviz_map_center_locked){
            marker.pose.position.x += map_pose.pose.position.x;
            marker.pose.position.y += map_pose.pose.position.y;
            marker.pose.position.z += map_pose.pose.position.z;
        }

        // Orientation
        Eigen::Quaternionf ori;
        ori.x() = ori.y() = ori.z() = 0.f;
        ori.w() = 1.f;

        float angle = atan2(c->envelope.vertexes[4].y - c->envelope.vertexes[2].y, c->envelope.vertexes[4].x - c->envelope.vertexes[2].x);
        Eigen::Quaternionf axis; //= quad * q1 * quad.inverse();
        axis.w() = cos(angle/2.f);
        axis.x() = 0.0;
        axis.y() = 0.0;
        axis.z() = sin(angle/2.f);
        Eigen::Quaternionf rotated_att = ori * axis;

        marker.pose.orientation.x = rotated_att.x();
        marker.pose.orientation.y = rotated_att.y();
        marker.pose.orientation.z = rotated_att.z();
        marker.pose.orientation.w = rotated_att.w();

        // Width, length, height
        marker.scale.x = sqrt((c->envelope.vertexes[2].x - c->envelope.vertexes[4].x)*(c->envelope.vertexes[2].x - c->envelope.vertexes[4].x)
                              + (c->envelope.vertexes[2].y - c->envelope.vertexes[4].y)*(c->envelope.vertexes[2].y - c->envelope.vertexes[4].y));
        marker.scale.y = sqrt((c->envelope.vertexes[2].x - c->envelope.vertexes[1].x)*(c->envelope.vertexes[2].x - c->envelope.vertexes[1].x)
                              + (c->envelope.vertexes[2].y - c->envelope.vertexes[1].y)*(c->envelope.vertexes[2].y - c->envelope.vertexes[1].y));

        marker.scale.z = c->envelope.vertexes[1].z - c->envelope.vertexes[0].z;
        marker_array.markers.push_back(marker);
    }

    // Add some useless cubics to remove old one.
    for(int extra_id = id; extra_id<50; ++extra_id){
        marker.id = id;
        ++id;

        marker.pose.position.x = -1000;
        marker.pose.position.y = -1000;
        marker.pose.position.z = 1000;
        marker.scale.x = 0;
        marker.scale.y = 0;
        marker.scale.z = 0;
        marker_array.markers.push_back(marker);
    }

    current_marker_pub.publish(marker_array);
}


void linesPublish(vector<Eigen::Vector3d> &points, int id, float r, float g, float b, float a, float width, int type=visualization_msgs::Marker::POINTS, bool clear_path = false)
{

    if(clear_path){
        points.clear();
        Eigen::Vector3d p;
        p << -1000.f, -1000.f, -1000.f;
        for(int i=1; i<100; ++i){
            points.push_back(p);
        }
    }else{
        if(points.empty()) {
            ROS_INFO("Empty path !");
            return;
        }
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "points_and_lines";
    marker.id = id;

    marker.scale.x = width;
    marker.scale.y = width;
    marker.scale.z = width;

    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.orientation.w  = 1.0;


    for(const auto & point : points)
    {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
    }

    astar_result_pub.publish(marker);
}



void mapFutureStatusCallback(const std_msgs::Float32MultiArrayConstPtr &future_risk)
{
    future_risk_locked = true;
    for(int i=0; i<VOXEL_NUM; ++i){
        for(int j=0; j<RISK_MAP_NUMBER; ++j){
            future_risk_global[i][j] = future_risk->data[i*future_risk->layout.dim[0].stride + j];
        }
    }
    future_risk_locked = false;

    map_pose_global.pose.position.x = future_risk->data[VOXEL_NUM*RISK_MAP_NUMBER];
    map_pose_global.pose.position.y = future_risk->data[VOXEL_NUM*RISK_MAP_NUMBER + 1];
    map_pose_global.pose.position.z = future_risk->data[VOXEL_NUM*RISK_MAP_NUMBER + 2];

    future_risk_updated = true;
}


int getPointSpatialIndexInMap(const PVAYPoint &p, const Eigen::Vector3d &map_center){
    static const int z_change_storage_taken = MAP_WIDTH_VOXEL_NUM*MAP_LENGTH_VOXEL_NUM;  //Order: zyxt
    static const int y_change_storage_taken = MAP_LENGTH_VOXEL_NUM;
    static const int x_change_storage_taken = 1;
    static const int max_size = MAP_HEIGHT_VOXEL_NUM*MAP_WIDTH_VOXEL_NUM*MAP_LENGTH_VOXEL_NUM;

    static const float map_length_half = MAP_LENGTH_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;
    static const float map_width_half  = MAP_WIDTH_VOXEL_NUM  * VOXEL_RESOLUTION / 2.f;
    static const float map_height_half = MAP_HEIGHT_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;

    auto x_index = (int)((p.position.x()-map_center.x() + map_length_half) / VOXEL_RESOLUTION);
    auto y_index = (int)((p.position.y()-map_center.y() + map_width_half) / VOXEL_RESOLUTION);
    auto z_index = (int)((p.position.z()-map_center.z() + map_height_half) / VOXEL_RESOLUTION);

    int index = z_index*z_change_storage_taken + y_index*y_change_storage_taken + x_index*x_change_storage_taken;
    if(index >= 0 && index < max_size){
        return index;
    }else{
        return -1;
    }
}


void trajectoryCallback(const ros::TimerEvent& e)
{
    static double last_end_time = ros::Time::now().toSec();

    ROS_WARN("Time interval between two plannings = %lf", ros::Time::now().toSec() - last_end_time);

    if(!future_risk_updated) return;

    double trajectory_planning_start_time = ros::Time::now().toSec();

    /// Copy future status
    static float future_risk_planning[VOXEL_NUM][RISK_MAP_NUMBER];
    while(future_risk_locked){
        ros::Duration(0.0001).sleep();
    }
    future_risk_locked = true;
    for(int i=0; i<VOXEL_NUM; ++i){
        for(int j=0; j<RISK_MAP_NUMBER; ++j){
            future_risk_planning[i][j] = future_risk_global[i][j];
        }
    }
    future_risk_locked = false;


    /***** P1: Check the risk of the planned short trajectory and set a start position ****/
    Eigen::Vector3d planning_start_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d planning_start_v = Eigen::Vector3d::Zero(); // uav_velocity_global;
    Eigen::Vector3d planning_start_a = Eigen::Vector3d::Zero(); // uav_velocity_global;

    Eigen::Vector3d planning_start_map_center;
    planning_start_map_center << map_pose_global.pose.position.x, map_pose_global.pose.position.y, map_pose_global.pose.position.z;

    geometry_msgs::PointStamped tracking_error_signal; // 0: normal 1: tracking error too large 2: safety mode
    tracking_error_signal.header.stamp = ros::Time::now();
    tracking_error_signal.point.x = 0.0;

    if(in_safety_mode){
        planning_start_p = p_store_for_em - planning_start_map_center;

        planning_start_v = Eigen::Vector3d::Zero();
        tracking_error_signal.point.x = 2.0;

    }else{
        /// Calculated risk of planned trajectory
        float risk = 0.f;
        std::queue<PVAYPoint> current_queue_copy(trajectory_piece);
        while (!current_queue_copy.empty())
        {
            auto p = current_queue_copy.front();

            int spatial_index = getPointSpatialIndexInMap(p, planning_start_map_center);
            if(spatial_index >= 0){
                risk += future_risk_global[spatial_index][0];
            }

            current_queue_copy.pop();
        }

        /// Set planning initial state
        if(risk > risk_threshold_motion_primitive){
            planning_start_p = p_store_for_em - planning_start_map_center;
            // Empty the queue so it will enter safety mode
            std::queue<PVAYPoint> empty;
            std::swap( trajectory_piece, empty);
            ROS_WARN("Current planned trajecotry not safe!");
        }
        else if(!trajectory_piece.empty() && (p_store_for_em - uav_position_global).norm() > 1.0){
            // Tracking error too large
            PVAYPoint temp_p;
            temp_p.position = 0.5*p_store_for_em + 0.5*uav_position_global;
            temp_p.velocity = Eigen::Vector3d::Zero();
            temp_p.acceleration = Eigen::Vector3d::Zero();
            temp_p.yaw = 0.0;

            // Empty the queue and add the temp point
            std::queue<PVAYPoint> empty;
            std::swap( trajectory_piece, empty);
            trajectory_piece.push(temp_p);

            planning_start_p = temp_p.position - planning_start_map_center;

            ROS_WARN("Tracking error too large. Set a new start point.");
            tracking_error_signal.point.x = 1.0;
        }
        else{
            if(trajectory_piece.size() > trajectory_piece_max_size * 0.8)
            {
                /// Planning is not necessary
                ROS_WARN("Planning is not necessary!");
                last_end_time = ros::Time::now().toSec();
                return;
            }
            else if(!trajectory_piece.empty()){
                planning_start_p = trajectory_piece.back().position - planning_start_map_center;
                planning_start_v = trajectory_piece.back().velocity;
                planning_start_a = trajectory_piece.back().acceleration;
                if(planning_start_a.norm() > max_differentiated_current_a){
                    planning_start_a = planning_start_a / planning_start_a.norm() * max_differentiated_current_a;
                }
            }
        }
    }
    tracking_error_too_large_state_pub.publish(tracking_error_signal);


    /***** P2: Risk-aware Kino-dynamic A*  ****/
    double astar_start_time = ros::Time::now().toSec();

    if(fabs(planning_start_v.x()) > astar_planner.v_max_xy){//} || fabs(planning_start_v.y()) > astar_planner.v_max_xy || fabs(planning_start_v.z()) > astar_planner.v_max_z){
        planning_start_v.x() = astar_planner.v_max_xy * planning_start_v.x() / fabs(planning_start_v.x());
    }
    if(fabs(planning_start_v.y()) > astar_planner.v_max_xy){
        planning_start_v.y() = astar_planner.v_max_xy * planning_start_v.y() / fabs(planning_start_v.y());
    }
    if(fabs(planning_start_v.z()) > astar_planner.v_max_z){
        planning_start_v.z() = astar_planner.v_max_z * planning_start_v.z() / fabs(planning_start_v.z());
    }

    Node *start_node = new Node(0, planning_start_p.x(), planning_start_p.y(), planning_start_p.z(),
                                planning_start_v.x(), planning_start_v.y(), planning_start_v.z());
    Node *end_node = new Node(0, goal_x-planning_start_map_center(0),goal_y-planning_start_map_center(1),goal_z-planning_start_map_center(2), 0, 0, 0);
    vector<Node*> result;

    float start_time = trajectory_piece.size() * planning_time_step; //start time


    astar_planner.updateMapCenterPosition(planning_start_map_center(0), planning_start_map_center(1), planning_start_map_center(2));
    astar_planner.search(start_node, end_node, start_time, expand_safety_distance, reference_direction_angle, &future_risk_planning[0][0], result); //distance = 0.25

    vector<TrajPoint> searched_points;
    astar_planner.getSearchedPoints(searched_points);


    /// Visualize searched points
//    vector<Eigen::Vector3d> searched_points_to_show;
//    for(const auto &searched_point : searched_points){
//        Eigen::Vector3d p;
//        if(rviz_map_center_locked){
//            p<<searched_point.x, searched_point.y, searched_point.z;
//        }else{
//            p<<searched_point.x+map_pose_global.pose.position.x, searched_point.y+map_pose_global.pose.position.y, searched_point.z+map_pose_global.pose.position.z;
//        }
//        searched_points_to_show.push_back(p);
//    }
//
//    linesPublish(searched_points_to_show, 89, 1.0, 0.1, 0.1, 1.0, 0.05);


    /// To record max and avg search time
    static double a_star_total_time = 0.0;
    static double a_star_max_time = 0.0;
    static int a_star_counter = 0;
    double a_star_time = ros::Time::now().toSec() - astar_start_time;
    a_star_total_time += a_star_time;
    a_star_counter += 1;

    if(a_star_time > a_star_max_time){
        a_star_max_time = a_star_time;
    }

    ROS_INFO("A* time = %lf", a_star_time);
    ROS_INFO("A* AVG time = %lf", a_star_total_time / (double)a_star_counter );
    ROS_INFO("A* MAX time = %lf", a_star_max_time);


    if(result.size() > 1 && result.size() < 10){ //at least two nodes to build a corridor

        // Publish nodes
        vector<Eigen::Vector3d> points;
        for(auto &p : result){
            Eigen::Vector3d p_this;
            if(rviz_map_center_locked) {
                p_this.x() = p->x;
                p_this.y() = p->y;
                p_this.z() = p->z;
            }else{
                p_this.x() = p->x + map_pose_global.pose.position.x;
                p_this.y() = p->y + map_pose_global.pose.position.y;
                p_this.z() = p->z + map_pose_global.pose.position.z;
            }

            points.push_back(p_this);
        }
        linesPublish(points, 0, 0.8, 0.3, 0.4, 1.0, 0.2);

        // Set reference_direction_angle
        reference_direction_angle = atan2(points[1].y()-points[0].y(), points[1].x()-points[0].x());

        // Restore trajectories and publish
        vector<Eigen::Vector3d> a_star_traj_points_to_show;

        queue<PVAYPoint> trajectory_piece_temp;

        for(int i=0; i<result.size()-1; ++i){
            auto node1 = result[i];
            auto node2 = result[i+1];

            float ax = (node2->vx - node1->vx) / astar_planner.time_step_node;
            float ay = (node2->vy - node1->vy) / astar_planner.time_step_node;
            float az = (node2->vz - node1->vz) / astar_planner.time_step_node;
            auto point_num_one_piece = (int)(astar_planner.time_step_node / astar_planner.time_step_trajectory);

            for(int j=1; j<point_num_one_piece; ++j){ //Skip the first point. Which is the same as the last point on the last piece.
                Eigen::Vector3d p;
                float t = (float)j*astar_planner.time_step_trajectory;
                p.x() = node1->x + node1->vx*t + 0.5*ax*t*t;
                p.y() = node1->y + node1->vy*t + 0.5*ay*t*t;
                p.z() = node1->z + node1->vz*t + 0.5*az*t*t;

                if(!rviz_map_center_locked){
                    p.x() += map_pose_global.pose.position.x;
                    p.y() += map_pose_global.pose.position.y;
                    p.z() += map_pose_global.pose.position.z;
                }

                a_star_traj_points_to_show.push_back(p);

            }

        }
        linesPublish(a_star_traj_points_to_show, 1, 0.1, 0.9, 0.2, 1.0, 0.1, visualization_msgs::Marker::LINE_STRIP);


        /***** P3: Risk-constrained corridor ****/
        double corridor_start_time = ros::Time::now().toSec();
        vector<Corridor*> corridors;

        astar_planner.findCorridors(corridors, 2);


        /// To record max and avg corridors calculation time
        static double corridor_total_time = 0.0;
        static double corridor_max_time = 0.0;
        static int corridor_counter = 0;
        double corridor_time = ros::Time::now().toSec() - corridor_start_time;
        corridor_total_time += corridor_time;
        corridor_counter += 1;

        if(corridor_time > corridor_max_time){
            corridor_max_time = corridor_time;
        }

        ROS_INFO("corridors time this = %lf",corridor_time);
        ROS_INFO("corridors AVG time = %lf", corridor_total_time / (double)corridor_counter );
        ROS_INFO("corridors MAX time = %lf", corridor_max_time);

        /// Publish corridors to optimization planner
        decomp_ros_msgs::DynPolyhedronArray corridor_msg;
        corridor_msg.header.stamp = ros::Time::now();
        corridor_msg.start_pos.x = result[0]->x;
        corridor_msg.start_pos.y = result[0]->y;
        corridor_msg.start_pos.z = result[0]->z;
        corridor_msg.start_vel.x = result[0]->vx;
        corridor_msg.start_vel.y = result[0]->vy;
        corridor_msg.start_vel.z = result[0]->vz;
        corridor_msg.start_acc.x = planning_start_a.x();
        corridor_msg.start_acc.y = planning_start_a.y();
        corridor_msg.start_acc.z = planning_start_a.z();

        corridor_msg.end_pos.x = result[result.size()-1]->x;
        corridor_msg.end_pos.y = result[result.size()-1]->y;
        corridor_msg.end_pos.z = result[result.size()-1]->z;
        corridor_msg.end_vel.x = result[result.size()-1]->vx;
        corridor_msg.end_vel.y = result[result.size()-1]->vy;
        corridor_msg.end_vel.z = result[result.size()-1]->vz;

        corridor_msg.end_acc.x = 0.f;
        corridor_msg.end_acc.y = 0.f;
        corridor_msg.end_acc.z = 0.f;


        for(auto & corridor : corridors)
        {
            decomp_ros_msgs::DynPolyhedron corridor_this;
            corridor_this.duration = a_star_search_time_step;
            for(const auto &surface : corridor->envelope.surfaces)
            {
                geometry_msgs::Point point;
                geometry_msgs::Point normal;
                point.x = surface.point.x;
                point.y = surface.point.y;
                point.z = surface.point.z;
                normal.x = surface.normal.x;
                normal.y = surface.normal.y;
                normal.z = surface.normal.z;
                corridor_this.points.push_back(point);
                corridor_this.normals.push_back(normal);
            }

            corridor_msg.dyn_polyhedrons.push_back(corridor_this);
        }
        corridor_pub.publish(corridor_msg);
        /// Publish corridors to RVIZ
        corridorsPublish(corridors, map_pose_global);


        /***** P4: Trajectory Optimization *****/
        double optimization_start_t = ros::Time::now().toSec();
        bool trajectory_optimized = optimizationInCorridors(corridor_msg, planning_start_map_center);
        ROS_INFO("optimization time = %lf", ros::Time::now().toSec() - optimization_start_t);
        if(!trajectory_optimized){
            ROS_WARN("No optimization result!!");
        }

    }else{ // A* no result
        /// Eliminate the left points in RVIZ
        vector<Eigen::Vector3d> points;
        linesPublish(points, 0, 0.8, 0.3, 0.4, 1.0, 0.2, visualization_msgs::Marker::POINTS, true);
        linesPublish(points, 1, 0.1, 0.9, 0.2, 1.0, 0.1, visualization_msgs::Marker::LINE_STRIP, true);

        vector<Corridor*> corridors;
        corridorsPublish(corridors, map_pose_global, true);

//        /******** TEST code for emergency *********/
//        std::queue<PVAYPoint> empty;
//        std::swap( trajectory_piece, empty);
    }

    ROS_INFO("Planning thread total time = %lf", ros::Time::now().toSec() - trajectory_planning_start_time);
    last_end_time = ros::Time::now().toSec();
}


/**
 * @brief display colorful velocity on trajectories
 *
 * @param a
 * @return Eigen::Vector3d
 */
inline Eigen::Vector3d jetColor(double a) {
    double s = a * 4;
    Eigen::Vector3d c;  // [r, g, b]
    switch ((int)floor(s)) {
        case 0:
            c << 0, 0, s;
            break;
        case 1:
            c << 0, s - 1, 1;
            break;
        case 2:
            c << s - 2, 1, 3 - s;
            break;
        case 3:
            c << 1, 4 - s, 0;
            break;
        default:
            c << 1, 0, 0;
            break;
    }
    return c;
}


/**
 * @brief visualize trajectory
 *
 * @param appliedTraj
 * @param maxV
 */
void visualizeTraj(const Trajectory& appliedTraj, const Eigen::Vector3d &planning_start_map_center, double maxV) {
    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = ros::Time::now();
    traj_marker.type = visualization_msgs::Marker::LINE_LIST;
    traj_marker.pose.orientation.w = 1.00;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.id = 0;
    traj_marker.ns = "trajectory";
    traj_marker.color.r = 0.00;
    traj_marker.color.g = 0.50;
    traj_marker.color.b = 1.00;
    traj_marker.scale.x = 0.10;

    double T = 0.05;
    Eigen::Vector3d lastX = appliedTraj.getPos(0.0) + planning_start_map_center;
    for (double t = T; t < appliedTraj.getDuration(); t += T) {
        std_msgs::ColorRGBA c;
        Eigen::Vector3d jets = jetColor(appliedTraj.getVel(t).norm() / maxV);
        c.r = jets[0];
        c.g = jets[1];
        c.b = jets[2];
        c.a = 0.8;

        geometry_msgs::Point point;
        Eigen::Vector3d X = appliedTraj.getPos(t) + planning_start_map_center;
        point.x = lastX(0);
        point.y = lastX(1);
        point.z = lastX(2);
        traj_marker.points.push_back(point);
        traj_marker.colors.push_back(c);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        traj_marker.points.push_back(point);
        traj_marker.colors.push_back(c);
        lastX = X;
    }
    color_vel_pub.publish(traj_marker);
}


/**
 * @brief extract corridor from messages
 * @param msgs
 * @return std::vector<Eigen::Matrix<double, 6, -1>>
 */
std::vector<Eigen::Matrix<double, 6, -1>> dynPolyArrayToVector(
        const decomp_ros_msgs::DynPolyhedronArray& msgs) {
    std::vector<Eigen::Matrix<double, 6, -1>> polyhedra;
    polyhedra.reserve(msgs.dyn_polyhedrons.size());
    for (const auto& v : msgs.dyn_polyhedrons) {
        Eigen::MatrixXd polygon;
        polygon.resize(6, v.points.size());
        for (unsigned int i = 0; i < v.points.size(); i++) {
            Eigen::Vector3d p(v.points[i].x, v.points[i].y, v.points[i].z);
            Eigen::Vector3d n(v.normals[i].x, v.normals[i].y, v.normals[i].z);
            polygon.col(i).tail<3>() = p;
            polygon.col(i).head<3>() = n;
        }
        polyhedra.push_back(polygon);
    }
    return polyhedra;
}

/**
 * @brief extract time allocation from message
 * timestamp in msg is the time when uav leaves the corridor
 * @param msgs
 * @return std::vector<double>
 */
std::vector<double> dynPolyArrayToTimeAlloc(
        const decomp_ros_msgs::DynPolyhedronArray& msgs) {
    std::vector<double> time_allocations;
    for (const auto& v: msgs.dyn_polyhedrons) {
        time_allocations.push_back(v.duration);
    }

    return time_allocations;
}

/**
 * @brief extract initial position from msg
 * @param msg
 * @return Eigen::Matrix3d [pos, vel, acc]
 */
Eigen::Matrix3d dynPolyArrayToInitPos(
        const decomp_ros_msgs::DynPolyhedronArray& msg) {
    Eigen::Matrix3d start;
    start.col(0) << msg.start_pos.x, msg.start_pos.y, msg.start_pos.z;
    start.col(1) << msg.start_vel.x, msg.start_vel.y, msg.start_vel.z;
    start.col(2) << msg.start_acc.x, msg.start_acc.y, msg.start_acc.z;
    return start;
}

/**
 * @brief extract end position from msg
 * @param msg
 * @return Eigen::Matrix3d [pos, vel, acc]
 */
Eigen::Matrix3d dynPolyArrayToEndPos(
        const decomp_ros_msgs::DynPolyhedronArray& msg) {
    Eigen::Matrix3d end;
    end.col(0) << msg.end_pos.x, msg.end_pos.y, msg.end_pos.z;
    end.col(1) << msg.end_vel.x, msg.end_vel.y, msg.end_vel.z;
    end.col(2) << msg.end_acc.x, msg.end_acc.y, msg.end_acc.z;
    return end;
}



ros::Time traj_start_;
ros::Time traj_end_;
bool optimizationInCorridors(const decomp_ros_msgs::DynPolyhedronArray msg, const Eigen::Vector3d planning_start_map_center) {
    auto corridors = dynPolyArrayToVector(msg);
    auto time_alloc = dynPolyArrayToTimeAlloc(msg);

    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();


    /* get initial states and end states */
    Eigen::Vector3d zero(0.0, 0.0, 0.0);
    Eigen::Matrix3d init_state = dynPolyArrayToInitPos(msg);
    Eigen::Matrix3d finl_state = dynPolyArrayToEndPos(msg);
    std::cout << "init\t" << init_state << std::endl;
    std::cout << "final\t" << finl_state << std::endl;


    /* clean buffer */

    double T = 0;
    for (auto it = time_alloc.begin(); it != time_alloc.end(); ++it) {
        T += (*it);
    }
    std::cout << "Time Size: " << time_alloc.size() << std::endl;
    std::cout << "Time: " << T << std::endl;

    mini_snap_.reset(init_state, finl_state, time_alloc, corridors);

    bool is_solved = false;
    try{
        is_solved = mini_snap_.optimize(factors, delta_corridor);
    }catch(int e) {
        ROS_ERROR("Optimizer crashed!");
        return false;
    }

    if (!is_solved) {
        ROS_ERROR("No solution found for these corridors!");
        return false;
    }

    mini_snap_.getTrajectory(&traj_);
    int I = 10;  // max iterations
    int i = 0;
    while (!mini_snap_.isCorridorSatisfied(traj_, max_vel_optimization, max_acc_optimization, delta_corridor) && i++ < I) {
//        std::cout << "out of corridor:\t" << i << std::endl;
        try{
            is_solved = mini_snap_.reOptimize();
        }catch(int e) {
            ROS_ERROR("Optimizer crashed!");
            return false;
        }

        if(is_solved){
            mini_snap_.getTrajectory(&traj_);
        }else{
            ROS_ERROR("No solution found for these corridors!");
            return false;
        }
    }  // apply minimum snap optimization



    /// Add to queue
    float time = 0.f;
    trajectory_to_nmpc = trajectory_piece;

    static int buffer_size = nmpc_receive_points_num*2;
    while(trajectory_to_nmpc.size() <= buffer_size && time < T)
    {
        time += planning_time_step;
        PVAYPoint p;
        p.position = traj_.getPos(time) + planning_start_map_center; // map frame to global frame
        p.velocity = traj_.getVel(time);
        p.acceleration = traj_.getAcc(time);

        /** Yaw direction planning: velocity direction **/
        double yaw_sp = atan2(p.velocity.y(), p.velocity.x());
        static double max_delt_yaw = 0.05, max_yaw = 0.7, yaw_sp_last = 0.0;
        if(yaw_sp > yaw_sp_last + max_delt_yaw){
            yaw_sp = yaw_sp_last + max_delt_yaw;
        }else if(yaw_sp < yaw_sp_last - max_delt_yaw){
            yaw_sp = yaw_sp_last - max_delt_yaw;
        }
        if(yaw_sp > max_yaw){
            yaw_sp = max_yaw;
        }else if(yaw_sp < -max_yaw){
            yaw_sp = -max_yaw;
        }

        yaw_sp_last = yaw_sp;
        p.yaw = yaw_sp;
        trajectory_to_nmpc.push(p);

        if(trajectory_piece.size() < trajectory_piece_max_size){
            trajectory_piece.push(p);
        }
    }
    trajectory_initialized = true;


    static double time_last_planned = ros::Time::now().toSec();
    double time_from_last_valid_planning =  ros::Time::now().toSec()-time_last_planned;
    if(time_from_last_valid_planning < 0.15){
        ROS_INFO("Time from last valid planning is = %lf s", time_from_last_valid_planning);
    }else{
        ROS_ERROR("Time from last valid planning is = %lf s", time_from_last_valid_planning);
    }
    time_last_planned = ros::Time::now().toSec();

    queue<PVAYPoint> current_queue = trajectory_piece;
    vector<Eigen::Vector3d> queue_points_to_show;
    while(!current_queue.empty()){
        PVAYPoint p = current_queue.front();
        current_queue.pop();
        Eigen::Vector3d position_this;
        position_this << p.position.x(), p.position.y(), p.position.z();

        if(rviz_map_center_locked){position_this -= planning_start_map_center;}

        queue_points_to_show.push_back(position_this);
    }

    linesPublish(queue_points_to_show, 45, 0.5, 0.2, 0.8, 1.0, 0.1);


    /// Print and visualization
//    std::cout << "\033[35m EVALUATIONS" << std::endl;
//    std::cout << " number of pieces: " << time_alloc.size() << std::endl;
//    std::cout << "[TrajOpt] Iterations: " << i << std::endl;
//
//    std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
//    std::cout << "[TrajOpt] Computation time: "
//              << 1.0e-3 * std::chrono::duration_cast<std::chrono::microseconds>(
//                      toc - tic)
//                      .count()
//              << "ms" << std::endl;
//    std::cout << "[TrajOpt] Final cost: " << mini_snap_.getMinimumCost()
//              << std::endl;
//    std::cout << "[TrajOpt] Max velocity: " << traj_.getMaxVelRate() << std::endl;
//    std::cout << "[TrajOpt] Max acclerate: " << traj_.getMaxAccRate()
//              << std::endl;
//    std::cout << "[TrajOpt] Total time: " << traj_.getDuration() << std::endl;
//    std::cout << " \033[0m" << std::endl;

    // initialize visualization
//    nav_msgs::Path path;
//    double dt = 0.05;
//    traj_start_ = ros::Time::now();              // start timestamp
//    traj_end_ = traj_start_ + ros::Duration(T);  // end timestamp
//
//    path.header.frame_id = "map";
//    path.header.stamp = traj_start_;
//
//    for (double t = 0.0; t < T; t += dt) {
//        geometry_msgs::PoseStamped point;
//        point.header.frame_id = "map";
//        point.header.stamp = traj_start_ + ros::Duration(t);
//        Eigen::Vector3d pos = traj_.getPos(t);
//        point.pose.position.x = pos(0) + planning_start_map_center.x();
//        point.pose.position.y = pos(1) + planning_start_map_center.y();
//        point.pose.position.z = pos(2) + planning_start_map_center.z();
//        point.pose.orientation.w = 1;
//        point.pose.orientation.x = 0;
//        point.pose.orientation.y = 0;
//        point.pose.orientation.z = 0;
//        path.poses.push_back(point);
//    }
//    trajectory_pub.publish(path);

    visualizeTraj(traj_, planning_start_map_center, 3.0);
    corridors.clear();

    return  true;
}

void setpointCallback(const ros::TimerEvent& e)
{
    if(!position_received || !trajectory_initialized){
        return;
    }

    static bool p_store_for_em_initialized = false;
    if(!p_store_for_em_initialized){
        p_store_for_em = uav_position_global;
        p_store_for_em_initialized = true;
    }

    static double flight_start_time = 0.0;


    trajectory_msgs::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();

    /// Add current point
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    geometry_msgs::Transform tr;
    tr.translation.x = uav_position_global.x();
    tr.translation.y = uav_position_global.y();
    tr.translation.z = uav_position_global.z();
    tr.rotation.w = 1.0; ///Yaw zero for the moment
    tr.rotation.x = 0;
    tr.rotation.y = 0;
    tr.rotation.z = 0;
    point.transforms.push_back(tr);

    geometry_msgs::Twist vel;
    vel.linear.x = uav_velocity_global.x();
    vel.linear.y = uav_velocity_global.y();
    vel.linear.z = uav_velocity_global.z();
    point.velocities.push_back(vel);
    traj_msg.points.push_back(point);

    if(!trajectory_piece.empty())  // Trajectory available
    {
        in_safety_mode = false;

        auto trajectory_to_nmpc_copy = trajectory_to_nmpc;
        trajectory_to_nmpc.pop();

        while(traj_msg.points.size() < 20 && !trajectory_to_nmpc_copy.empty())
        {
            auto p = trajectory_to_nmpc_copy.front();
            trajectory_to_nmpc_copy.pop();

            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::Transform tr;
            tr.translation.x = p.position.x();
            tr.translation.y = p.position.y();
            tr.translation.z = p.position.z();
            tr.rotation.w = 1.0; ///Yaw zero for the moment
            tr.rotation.x = 0;
            tr.rotation.y = 0;
            tr.rotation.z = 0;
            point.transforms.push_back(tr);

            geometry_msgs::Twist vel;
            vel.linear.x = p.velocity.x();
            vel.linear.y = p.velocity.y();
            vel.linear.z = p.velocity.z();
            point.velocities.push_back(vel);
            traj_msg.points.push_back(point);
        }

        p_store_for_em << trajectory_piece.front().position.x(), trajectory_piece.front().position.y(), trajectory_piece.front().position.z();
        v_store_for_em << trajectory_piece.front().velocity.x(), trajectory_piece.front().velocity.y(), trajectory_piece.front().velocity.z();
        a_store_for_em << trajectory_piece.front().acceleration.x(), trajectory_piece.front().acceleration.y(), trajectory_piece.front().acceleration.z();

        yaw_store_for_em = trajectory_piece.front().yaw;

        trajectory_piece.pop();

        static double recorded_start_time = ros::Time::now().toSec();
        flight_start_time = recorded_start_time;

    }else{
        /// Safety mode
        in_safety_mode = true;

        /// TEST code: let hover height in safety mode be 1.0
        p_store_for_em.z() = 1.1;

        ROS_WARN("No available trajectory point. Safety mode!");
        while(traj_msg.points.size() < 20)
        {
            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::Transform tr;
            tr.translation.x = p_store_for_em.x();
            tr.translation.y = p_store_for_em.y();
            tr.translation.z = p_store_for_em.z();
            tr.rotation.w = 1.0; ///Yaw zero for the moment
            tr.rotation.x = 0;
            tr.rotation.y = 0;
            tr.rotation.z = 0;
            point.transforms.push_back(tr);

            geometry_msgs::Twist vel;
            vel.linear.x = 0.0;
            vel.linear.y = 0.0;
            vel.linear.z = 0.0;
            point.velocities.push_back(vel);
            traj_msg.points.push_back(point);

            v_store_for_em << 0, 0, 0;
            a_store_for_em << 0, 0, 0;
        }
    }

    /// Add predicted points if no enough points for mpc
    geometry_msgs::Transform tr_last = traj_msg.points[traj_msg.points.size()-1].transforms[0];
    geometry_msgs::Twist vel_last = traj_msg.points[traj_msg.points.size()-1].velocities[0];

    int counter = 0;
    while(traj_msg.points.size() < 20)
    {
        counter ++;
        geometry_msgs::Transform tr_this = tr_last;
        tr_this.rotation.x += counter * planning_time_step;
        tr_this.rotation.y += counter * planning_time_step;
        geometry_msgs::Twist vel_this = vel_last;
        vel_this.linear.z = 0.0;

        trajectory_msgs::MultiDOFJointTrajectoryPoint point_this;
        point_this.transforms.push_back(tr_this);
        point_this.velocities.push_back(vel_this);

        traj_msg.points.push_back(point_this);
    }

    pva_traj_pub.publish(traj_msg);

    /// Publish the first control points to show tracking error
    geometry_msgs::TwistStamped velocity_sp_msg;
    velocity_sp_msg.header.stamp = ros::Time::now();
    velocity_sp_msg.twist.linear.x = v_store_for_em(0);
    velocity_sp_msg.twist.linear.y = v_store_for_em(1);
    velocity_sp_msg.twist.linear.z = v_store_for_em(2);
    velocity_setpoint_pub.publish(velocity_sp_msg);


    geometry_msgs::PoseStamped pose_sp_msg;
    pose_sp_msg.header.stamp = ros::Time::now();
    pose_sp_msg.pose.position.x = p_store_for_em(0) - uav_position_global.x();
    pose_sp_msg.pose.position.y = p_store_for_em(1) - uav_position_global.y();
    pose_sp_msg.pose.position.z = p_store_for_em(2) - uav_position_global.z();
    position_tracking_pub.publish(pose_sp_msg);


    /// Publish the pva setpoint.
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;
    pva_setpoint.positions.push_back(p_store_for_em.x()); //x
    pva_setpoint.positions.push_back(p_store_for_em.y()); //y
    pva_setpoint.positions.push_back(p_store_for_em.z()); //z
    pva_setpoint.positions.push_back(yaw_store_for_em);  //yaw

    pva_setpoint.velocities.push_back(v_store_for_em.x());
    pva_setpoint.velocities.push_back(v_store_for_em.y());
    pva_setpoint.velocities.push_back(v_store_for_em.z());

    pva_setpoint.accelerations.push_back(a_store_for_em.x());
    pva_setpoint.accelerations.push_back(a_store_for_em.y());
    pva_setpoint.accelerations.push_back(a_store_for_em.z());

    pva_pub.publish(pva_setpoint);


    /// Publish mode
    std_msgs::UInt8 planning_mode;
    if(in_safety_mode){
        planning_mode.data = 0;
    }else{
        planning_mode.data = 1;
    }
    mode_pub.publish(planning_mode);


    if(fabs(uav_position_global.x() - goal_x) < 2.f && fabs(uav_position_global.y() - goal_y) < 2.f)
    {
        static double flight_end_time = ros::Time::now().toSec();
        ROS_WARN_THROTTLE(1,"Flight finished, time = %lf ", flight_end_time - flight_start_time);

        ros::shutdown();
    }


}

static void split(const string& s, vector<string>& tokens, const string& delimiters = " ")
{
    string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (string::npos != pos || string::npos != lastPos) {
        tokens.push_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}


void simPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    if(!state_locked)
    {
        state_locked = true;
        uav_position_global.x() = msg.pose.position.x;
        uav_position_global.y() = msg.pose.position.y;
        uav_position_global.z() = msg.pose.position.z;

        uav_att_global.x() = msg.pose.orientation.x;
        uav_att_global.y() = msg.pose.orientation.y;
        uav_att_global.z() = msg.pose.orientation.z;
        uav_att_global.w() = msg.pose.orientation.w;

        uav_position_global_queue.push(uav_position_global);
        uav_att_global_queue.push(uav_att_global);
        pose_att_time_queue.push(msg.header.stamp.toSec());

        position_received = true;
    }

    state_locked = false;

    Eigen::Quaternionf axis; //= quad * q1 * quad.inverse();
    axis.w() = cos(-M_PI/4.0);
    axis.x() = 0.0;
    axis.y() = 0.0;
    axis.z() = sin(-M_PI/4.0);
    Eigen::Quaternionf rotated_att = uav_att_global * axis;
}


void simVelocityCallback(const geometry_msgs::TwistStamped &msg)
{
    uav_velocity_global.x() = msg.twist.linear.x;
    uav_velocity_global.y() = msg.twist.linear.y;
    uav_velocity_global.z() = msg.twist.linear.z;

    /** Calculate virtual accelerates from velocity. Original accelerates given by px4 is too noisy **/
    static bool init_v_flag = true;
    static double last_time, last_vx, last_vy, last_vz;

    if(init_v_flag){
        init_v_flag = false;
    }
    else{
        double delt_t = ros::Time::now().toSec() - last_time;
        uav_acceleration_global(0) = (uav_velocity_global(0) - last_vx) / delt_t;
        uav_acceleration_global(1) = (uav_velocity_global(1) - last_vy) / delt_t;
        uav_acceleration_global(2) = (uav_velocity_global(2) - last_vz) / delt_t;

        if(fabs(uav_acceleration_global(0)) < 0.2) uav_acceleration_global(0) = 0.0;  //dead zone for acc x
        if(fabs(uav_acceleration_global(1)) < 0.2) uav_acceleration_global(1) = 0.0; //dead zone for acc y
        if(fabs(uav_acceleration_global(2)) < 0.2) uav_acceleration_global(2) = 0.0; //dead zone for acc z

        for(int i=0; i<3; i++){
            if(uav_acceleration_global(i) < -max_differentiated_current_a){
                uav_acceleration_global(i) = -max_differentiated_current_a;
            }else if(uav_acceleration_global(i) > max_differentiated_current_a){
                uav_acceleration_global(i) = max_differentiated_current_a;
            }
        }

        //ROS_INFO("acc=(%f, %f, %f)", uav_acceleration_global(0), uav_acceleration_global(1), uav_acceleration_global(2));
    }

    last_time = ros::Time::now().toSec();
    last_vx = uav_velocity_global(0);
    last_vy = uav_velocity_global(1);
    last_vz = uav_velocity_global(2);

}


void getParameterList(const ros::NodeHandle& nh) {
    nh.getParam("/planning_node/p_goal_x", goal_x);
    nh.getParam("/planning_node/p_goal_y", goal_y);
    nh.getParam("/planning_node/p_goal_z", goal_z);
    nh.getParam("/planning_node/max_vel", max_vel);
    nh.getParam("/planning_node/max_acc", max_acc);

    nh.getParam("/planning_node/max_vel_optimization", max_vel_optimization);
    nh.getParam("/planning_node/max_acc_optimization", max_acc_optimization);
    nh.getParam("/planning_node/max_differentiated_current_a", max_differentiated_current_a);

    nh.getParam("/planning_node/use_height_limit", use_height_limit);
    nh.getParam("/planning_node/height_limit_max", height_limit_max);
    nh.getParam("/planning_node/height_limit_min", height_limit_min);
    nh.getParam("/planning_node/sample_z_acc", sample_z_acc);
    nh.getParam("/planning_node/expand_safety_distance", expand_safety_distance);
    nh.getParam("/planning_node/trajectory_piece_max_size", trajectory_piece_max_size);
    nh.getParam("/planning_node/nmpc_receive_points_num", nmpc_receive_points_num);

    nh.getParam("/planning_node/pos_factor", factors[0]);
    nh.getParam("/planning_node/vel_factor", factors[1]);
    nh.getParam("/planning_node/acc_factor", factors[2]);
    nh.getParam("/planning_node/jerk_factor", factors[3]);
    nh.getParam("/planning_node/snap_factor", factors[4]);
    nh.getParam("/planning_node/delta_corridor", delta_corridor);

    nh.getParam("/planning_node/planning_time_step", planning_time_step);
    nh.getParam("/planning_node/a_star_acc_sample_step", a_star_acc_sample_step);
    nh.getParam("/planning_node/a_star_search_time_step", a_star_search_time_step);

    nh.getParam("/planning_node/rviz_map_center_locked", rviz_map_center_locked);

    nh.getParam("/planning_node/risk_threshold_motion_primitive", risk_threshold_motion_primitive);
    nh.getParam("/planning_node/risk_threshold_single_voxel", risk_threshold_single_voxel);
    nh.getParam("/planning_node/risk_threshold_corridor", risk_threshold_corridor);

    ROS_WARN("Parameter loaded! height_limit_max=%f", height_limit_max);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle n;

    factors = {0,0.5,0.5,0,0};

    /// Set parameters
    getParameterList(n);
    astar_planner.setTimeParameters(a_star_search_time_step, planning_time_step);
    astar_planner.setHeightLimit(use_height_limit, height_limit_max, height_limit_min);
    astar_planner.setIfSampleZDirection(sample_z_acc);
    astar_planner.setMaximumVelAccAndStep(static_cast<float>(max_vel), static_cast<float>(max_vel), static_cast<float>(max_acc), static_cast<float>(max_acc/2.0), a_star_acc_sample_step);
    astar_planner.setRiskThreshold(risk_threshold_motion_primitive, risk_threshold_single_voxel, risk_threshold_corridor);

    ros::Subscriber future_risk_sub = n.subscribe("/my_map/future_risk_full_array", 1, mapFutureStatusCallback);
    ros::Subscriber pose_sub = n.subscribe("/mavros/local_position/pose", 1, simPoseCallback);
    ros::Subscriber vel_sub = n.subscribe("/mavros/local_position/velocity_local", 1, simVelocityCallback);

    current_marker_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 1);

    astar_result_pub = n.advertise<visualization_msgs::Marker>("/astar_result", 1);
    fov_pub = n.advertise<visualization_msgs::Marker>("/visualization_fov", 1);

    trajectory_pub = n.advertise<nav_msgs::Path>("/vis_waypoint_path", 1);

    corridor_pub = n.advertise<decomp_ros_msgs::DynPolyhedronArray>("/traj_opt/corridors", 1);


    color_vel_pub = n.advertise<visualization_msgs::Marker>("/colored_trajectory", 1);

    pva_traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 1, true);
    
    velocity_setpoint_pub = n.advertise<geometry_msgs::TwistStamped>("/command/velocity_setpoint", 1, true);
    position_tracking_pub = n.advertise<geometry_msgs::PoseStamped>("/command/position_tracking_error", 1, true);

    mode_pub = n.advertise<std_msgs::UInt8>("/traj_opt/mode", 1);
    tracking_error_too_large_state_pub = n.advertise<geometry_msgs::PointStamped>("/traj_opt/tracking_error_too_large_signal", 1);
    pva_pub = n.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1, true);


    ros::Duration(2).sleep();

    ros::Timer timer2 = n.createTimer(ros::Duration(planning_time_step), trajectoryCallback);
    ros::Timer timer3 = n.createTimer(ros::Duration(planning_time_step), setpointCallback);


    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
