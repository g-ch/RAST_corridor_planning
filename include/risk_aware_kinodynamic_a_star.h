//
// Created by clarence on 2021/12/16.
//

#include "dsp_map/map_parameters.h"
#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>

using namespace std;

typedef struct Point3D
{
    float x;
    float y;
    float z;
}Point3D;

typedef struct Plane3D
{
    Point3D point;
    Point3D normal;
}Plane3D;

typedef struct RectangleEnvelope
{
    float time_stamp_start = 0.f;
    float time_stamp_end = 0.f;
    vector<Plane3D> surfaces; // should have 6 surfaces
    vector<Point3D> vertexes; // should have 8 vertexes
}RectangleEnvelope;

typedef struct Node
{
    float x,y,z;
    float vx,vy,vz;
    float f; // total cost
    float g; // cost to start point
    float h; // cost to goal
    float time_stamp;
    long int coding_index;

    Node* father;
    RectangleEnvelope envelope;

    Node(float time_stamp, float x,float y,float z, float vx,float vy,float vz)
    {
        this->time_stamp = time_stamp;
        this->x = x;
        this->y = y;
        this->z = z;
        this->vx = vx;
        this->vy = vy;
        this->vz = vz;
        this->f = 0.f;
        this->g = 0.f;
        this->h = 0.f;
        this->father = nullptr;
    }
    Node(float time_stamp, float x,float y,float z, float vx,float vy,float vz, Node* father, RectangleEnvelope envelope_init)
    {
        this->time_stamp = time_stamp;
        this->x = x;
        this->y = y;
        this->z = z;
        this->vx = vx;
        this->vy = vy;
        this->vz = vz;
        this->f = 0.f;
        this->g = 0.f;
        this->h = 0.f;
        this->father = father;
        envelope = envelope_init;
    }
}Node;

typedef struct TrajPoint
{
    float time_stamp = 0.f;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float vx = 0.f;
    float vy = 0.f;
    float vz = 0.f;

}TrajPoint;


typedef struct Corridor
{
    RectangleEnvelope envelope;

    Node *node_start;
    Node *node_end;

    Corridor(Node *node_start, Node *node_end)
    {
        this->node_start = node_start;
        this->node_end = node_end;
    }

    Corridor()
    {
        this->node_start = nullptr;
        this->node_end = nullptr;
    }

}Corridor;


class Astar{
public:

    explicit Astar()
    {
        map_length_half = MAP_LENGTH_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;
        map_width_half  = MAP_WIDTH_VOXEL_NUM  * VOXEL_RESOLUTION / 2.f;
        map_height_half = MAP_HEIGHT_VOXEL_NUM * VOXEL_RESOLUTION / 2.f;

        time_step_node = 0.4;
        time_step_trajectory = 0.05;

        v_max_xy = 3.0;
        v_max_z = 3.0;

        // Initialize sample steps
        a_min_x = -2.0;
        a_min_y = -4.0;
        a_max_xy = 4.0;
        a_min_z = -2.0;
        a_max_z = 2.0;

        a_sample_step_xy = 2.0;
        a_sample_step_z = 2.0;

        boundary_width = 1.5;

        risk_limitation_motion_primitive = 0.15;
        risk_limitation_single_voxel = 0.15;
        risk_limitation_corridor = 2.0;

        setSampleVector();

        start_time = 0.f;

        height_max_limit = 0.f;
        height_min_limit = 0.f;
        use_height_limit = false;
        sample_z_acc = true;

        map_center_x = 0.f;
        map_center_y = 0.f;
        map_center_z = 0.f;

        safety_distance = 0.3f;

        reference_direction_angle = 100.f;

        std::cout << "Use Function search to search A* path" << std::endl;
    }

    ~Astar()= default;

    void setSampleVector(){
        a_sample_vector_x.clear();
        a_sample_vector_y.clear();
        a_sample_vector_z.clear();

        float a_x = a_min_x;
        while(a_x < a_max_xy + 0.001f){
            a_sample_vector_x.push_back(a_x);
            a_x += a_sample_step_xy;
        }

        // Sample y a little more aggressively
        float a_y = a_min_y - 0.5f;
        while(a_y < a_max_xy + 0.51f){
            a_sample_vector_y.push_back(a_y);
            a_y += a_sample_step_xy+0.5f;
        }

        if(sample_z_acc){
            float a_z = a_min_z;
            while(a_z < a_max_z + 0.001f){
                a_sample_vector_z.push_back(a_z);
                a_z += a_sample_step_z;
            }
        }else{
            a_sample_vector_z.push_back(0);
        }
    }


    void setTimeParameters(float time_step_node_to_set, float time_step_trajectory_to_set)
    {
        time_step_node = time_step_node_to_set;
        time_step_trajectory = time_step_trajectory_to_set;
    }

    void setIfSampleZDirection(bool if_sample_z_acc)
    {
        sample_z_acc = if_sample_z_acc;
    }

    void setHeightLimit(bool if_use_height_limit, float limit_max, float limit_min)
    {
        use_height_limit = if_use_height_limit;
        height_max_limit = limit_max;
        height_min_limit = limit_min;
        if(if_use_height_limit){
            cout << " \033[1;33m Kinodynamic A*: To use height limit. updateMapCenterPosition() must be operated every time. \033[0m" << endl;
        }
    }

    void setMaximumVelAccAndStep(float v_max_xy_set, float v_max_z_set, float a_max_xy_set, float a_max_z_set, float a_step_set)
    {
        v_max_xy = v_max_xy_set;
        v_max_z = v_max_z_set;

        a_min_x = -a_max_xy_set/2.f;
        a_min_y = -a_max_xy_set;
        a_max_xy = a_max_xy_set;
        a_min_z = -a_max_z_set;
        a_max_z = a_max_z_set;
        a_sample_step_xy = a_step_set;
        a_sample_step_z = a_step_set;

        setSampleVector();
    }


    void setRiskThreshold(float risk_motion_primitive, float risk_single_voxel, float risk_corridor)
    {
        risk_limitation_motion_primitive = risk_motion_primitive;
        risk_limitation_single_voxel = risk_single_voxel;
        risk_limitation_corridor = risk_corridor;
    }


    void updateMapCenterPosition(float map_px, float map_py, float map_pz)
    {
        map_center_x = map_px;
        map_center_y = map_py;
        map_center_z = map_pz;
    }


    void getSearchedPoints(vector<TrajPoint> &searched_points)
    {
        searched_points = searched_point_vector;
    }


    void search(Node* start_p_v_set, Node* end_pos_set, float start_time_this, float safety_distance_this, float reference_direction_angle_this, float *risk_map_in, vector<Node*> &result){
        start_node = start_p_v_set;
        end_node = end_pos_set;
        start_node->coding_index = 0;
        end_node->coding_index = 88888888;

        safety_distance = safety_distance_this;
        reference_direction_angle = reference_direction_angle_this;

        start_time = start_time_this;
        risk_map = risk_map_in;


        if(!checkNodeValid(start_node)){
            std::cout << "Invalid start point index or end point index." << std::endl;
            std::cout << "start_node px="<< start_node->x <<" py="<<start_node->y <<" pz=" << start_node->z << std::endl;
            return;
        }

        open_list.clear();
        close_list.clear();
        result_path.clear();
        result_path_reversed.clear();
        searched_point_vector.clear();

        start_node->f = 0.f;
        start_node->time_stamp = 0.f;
        open_list.push_back(start_node);

        Node* current_node;
        bool found_path = false;
        int step_counter = 0;

        while (!open_list.empty()){
            step_counter += 1;

            current_node = open_list[0];

            if(nodeEqual(current_node, end_node) || checkNodeOnBoundary(current_node, boundary_width) || step_counter > 300){
                if(checkNodeOnBoundary(current_node, boundary_width))
                {
                    cout << "Boundary condition reached !!!" <<endl;
                }
                // Reached goal or boundary
                addPath(current_node);

                // Reverse and Print
                for(int i=(int)result_path.size()-1; i>0; --i){
                    auto p = result_path[i];
                    result_path_reversed.push_back(p);
                    found_path = true;
//                    std::cout << "(" << p->x << ", " << p->y << ", " << p->z << ")" << std::endl;
                }

                result = result_path_reversed;
                break;
            }

            nextStep(current_node);

            close_list.push_back(current_node);
            open_list.erase(open_list.begin());

//            std::cout << "open_list length=" << open_list.size() << endl;
            sort(open_list.begin(),open_list.end(), compare);
        }

        if(found_path){
            cout << "Path found, searched times = "<<step_counter <<", result node size = " << result_path_reversed.size() << endl;
            cout << "x="<<start_node->x<<", y="<<start_node->y<<", z="<<start_node->z<<endl;

//            for(auto &n : result_path_reversed){
//                cout << "x="<<n->x<<", y="<<n->y<<", z="<<n->z<<endl;
//            }

        }else{
            cout << "Path not found, searched times = "<<step_counter << endl;
        }

    }

    bool checkIfAllTrueVector(const vector<bool> &vector)
    {
        for(const auto &v : vector){
            if(!v){
                return false;
            }
        }
        return true;
    }

    void findCorridors(vector<Corridor*> &corridors, int pattern = 1, float expand_step = 0.2)
    {
        /// Pattern 0: Expand the corridor to xyz directions at the same until the safety condition is not satisfied
        /// Pattern 1: Use Pattern 0 first. When Pattern 0 fails, close expanding direction z, and then y, and then x;
        /// Pattern 2: Expand by order {x_positive, x_negative, y_positive, y_negative, z_positive, z_negative} in each
        ///            iteration. A direction is banned when it is not safe.
        /// Pattern 3: Expand only to y and z. The rest is the same as Pattern 0.

        for(int node_seq=1; node_seq<result_path_reversed.size(); ++node_seq){
            auto* corridor_this = new Corridor(result_path_reversed[node_seq-1], result_path_reversed[node_seq]);
            corridor_this->envelope.time_stamp_start = result_path_reversed[node_seq-1]->time_stamp;
            corridor_this->envelope.time_stamp_end = result_path_reversed[node_seq]->time_stamp;
            corridor_this->node_start = result_path_reversed[node_seq-1];
            corridor_this->node_end = result_path_reversed[node_seq];

            /// Expand envelope of the motion primitive to find the corridor
            RectangleEnvelope envelope_expanded = result_path_reversed[node_seq]->envelope;

            vector<bool> expanding_direction = {true, true, true, true, true, true};
            vector<float> expanding_distance = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};


            vector<vector<bool>> expanding_order;
            vector<bool> expanding_direction2 = {true, true, true, true, false, false}; //xy
            vector<bool> expanding_direction3 = {true, true, false, false, false, false}; //x
            vector<bool> expanding_direction4 = {false, false, true, false, false, false}; //y+
            vector<bool> expanding_direction5 = {false, false, false, true, false, false}; //y-
            vector<bool> expanding_direction6 = {false, false, false, false, true, false}; //z+
            vector<bool> expanding_direction7 = {false, false, false, false, false, true}; //z-
            expanding_order.push_back(expanding_direction2);
            expanding_order.push_back(expanding_direction3);
            expanding_order.push_back(expanding_direction4);
            expanding_order.push_back(expanding_direction5);
            expanding_order.push_back(expanding_direction6);
            expanding_order.push_back(expanding_direction7);
            int pattern1_seq = 0;

            if(pattern == 3){
                expanding_direction[0] = expanding_direction[1] = false;
            }

            vector<bool> forbidden_direction = {false, false, false, false, false, false}; // for pattern 2
            vector<bool> all_false_vector = {false, false, false, false, false, false};
            int check_surface_seq = -1; // for pattern 2

            int step_counter = 0;
            while(step_counter < 30)
            {
                ++step_counter;

                if(pattern == 2){
                    check_surface_seq += 1;

                    for(int j=0; j<6; ++j){
                        int this_seq = check_surface_seq + j;
                        if(this_seq >= 6){
                            this_seq -= 6;
                        }
                        if(!forbidden_direction[this_seq]){
                            check_surface_seq = this_seq;
                            break;
                        }
                    }

                    expanding_direction = all_false_vector;
                    expanding_direction[check_surface_seq] = true;
                }

                // Expand to desired direction by one step
                for(int j=0; j<6; ++j){
                    if(expanding_direction[j]){
                        expanding_distance[j] += expand_step;
                    }
                }

                RectangleEnvelope envelope_expanded_this;
                expandEnvelope(result_path_reversed[node_seq]->envelope, envelope_expanded_this, expanding_distance);

                if(!checkIfEnvelopeSafe(envelope_expanded_this, risk_limitation_corridor, risk_limitation_single_voxel)){
                    for(int j=0; j<6; ++j){ // return to last step
                        if(expanding_direction[j]){
                            expanding_distance[j] -= expand_step;
                        }
                    }

                    if(pattern == 0){
                        break;
                    }else if(pattern == 1){
                        if(pattern1_seq < expanding_order.size()){
                            expanding_direction = expanding_order[pattern1_seq];
                            pattern1_seq ++;
                        }else{
                            break;
                        }

                    }else if(pattern == 2){
                        forbidden_direction[check_surface_seq] = true;
                        if(checkIfAllTrueVector(forbidden_direction)){
                            break;
                        }
                    }else{
                        break;
                    }

                }else{
                    envelope_expanded = envelope_expanded_this;
                }
            }

            corridor_this->envelope = envelope_expanded;
            corridors.push_back(corridor_this);
        }
    }


    void findNearestPoint2DOnALine(float A_x, float A_y, float B_x, float B_y, float P_x, float P_y, float &P_prime_x, float &P_prime_y)
    {
        float AB_x = B_x - A_x;
        float AB_y = B_y - A_y;
        float AP_x = P_x - A_x;
        float AP_y = P_y - A_y;
        float AB_length = sqrt(AB_x*AB_x + AB_y*AB_y);
        float AP_prime_length = (AP_x*AB_x + AP_y*AB_y) / AB_length;

        float scale_B = AP_prime_length / AB_length;
        P_prime_x = B_x*scale_B + A_x*(1-scale_B);
        P_prime_y = B_y*scale_B + A_y*(1-scale_B);
    }


    void rotate2DVector(float v_x, float v_y, float theta, float &v_rotated_x, float &v_rotated_y)
    {
        v_rotated_x = v_x*cos(theta) + v_y * sin(theta);
        v_rotated_y = -v_x*sin(theta) + v_y * cos(theta);
    }


    float vectorNorm(float x, float y, float z){
        return sqrt(x*x + y*y + z*z);
    }


    float vectorSquareNorm(float x, float y, float z){
        return x*x + y*y + z*z;
    }


    bool ifPointInEnvelope(float x, float y, float z, RectangleEnvelope &envelope)
    {
        for(const auto &e : envelope.surfaces)
        {
            float dot = (x - e.point.x)*e.normal.x + (y - e.point.y)*e.normal.y + (z - e.point.z)*e.normal.z;
//            if(dot > 0.f){ //outside
//                // check distance
//                float vector_xe_length = vectorNorm(x - e.point.x, y - e.point.y, z - e.point.z);
//                float angle = acos(dot/vector_xe_length); //vector normal was normalized
//                float outside_distance = vector_xe_length * sin(angle);
//
//                if(outside_distance > VOXEL_RESOLUTION){
//                    return false;
//                }
//            }
            if(dot > 0.f){
                return false;
            }
        }
        return true;
    }

    void expandEnvelope(RectangleEnvelope &envelope, RectangleEnvelope &envelope_expanded, vector<float> &searching_dists)
    {
        /// searching_directions[6]: [x_position, x_negative, y_position, y_negative, z_position, z_negative]

        float center_x = (envelope.vertexes[0].x + envelope.vertexes[2].x + envelope.vertexes[4].x + envelope.vertexes[6].x) / 4.f;
        float center_y = (envelope.vertexes[0].y + envelope.vertexes[2].y + envelope.vertexes[4].y + envelope.vertexes[6].y) / 4.f;
        float center_z = (envelope.vertexes[0].z + envelope.vertexes[1].z) / 2.f;

        float theta = M_PI / 2.f;
        if(envelope.vertexes[0].x-envelope.vertexes[2].x != 0.f)
        {
            theta = atan2(envelope.vertexes[0].y-envelope.vertexes[2].y, envelope.vertexes[0].x-envelope.vertexes[2].x);
        }

        for(auto &v : envelope.vertexes)
        {
            Point3D new_v;
            float v_rotated_x, v_rotated_y;
            rotate2DVector(v.x-center_x, v.y-center_y, theta, v_rotated_x, v_rotated_y);

            if(v_rotated_x >= 0.f){
                v_rotated_x += searching_dists[0];
            }else{
                v_rotated_x -= searching_dists[1];
            }
            if(v_rotated_y >= 0.f){
                v_rotated_y += searching_dists[2];
            }else{
                v_rotated_y -= searching_dists[3];
            }

            rotate2DVector(v_rotated_x, v_rotated_y, -theta, new_v.x, new_v.y);
            new_v.x += center_x;
            new_v.y += center_y;
            new_v.z = v.z;

            if(v.z - center_z >= 0){
                new_v.z = v.z + searching_dists[4];
            }else{
                new_v.z = v.z - searching_dists[5];
            }

            envelope_expanded.vertexes.push_back(new_v);
        }

        envelope_expanded.surfaces = envelope.surfaces;
        envelope_expanded.surfaces[0].point = envelope_expanded.vertexes[0];
        envelope_expanded.surfaces[1].point = envelope_expanded.vertexes[2];
        envelope_expanded.surfaces[2].point = envelope_expanded.vertexes[4];
        envelope_expanded.surfaces[3].point = envelope_expanded.vertexes[6];
        envelope_expanded.surfaces[4].point.z -= searching_dists[5];
        envelope_expanded.surfaces[5].point.z += searching_dists[4];

        envelope_expanded.time_stamp_start = envelope.time_stamp_start;
        envelope_expanded.time_stamp_end = envelope.time_stamp_end;
    }

    bool findRectangleEnvelope(Node *current_node, float ax, float ay, float az, float t, RectangleEnvelope &envelope)
    {
        float A_x, A_y, A_z, B_x, B_y, B_z, P_x, P_y, P_prime_x, P_prime_y;
        float A_prime_x, A_prime_y, B_prime_x, B_prime_y;

        A_x = current_node->x;
        A_y = current_node->y;
        A_z = current_node->z;
        B_x = A_x + current_node->vx*t + 0.5f*ax*t*t;
        B_y = A_y + current_node->vy*t + 0.5f*ay*t*t;
        B_z = A_z + current_node->vz*t + 0.5f*az*t*t;

        float tp = 0.5f*t;
        P_x = A_x + current_node->vx*tp + 0.5f*ax*tp*tp;
        P_y = A_y + current_node->vy*tp + 0.5f*ay*tp*tp;

        float PA_square_length = (P_x-A_x)*(P_x-A_x) + (P_y-A_y)*(P_y-A_y);
        float PB_square_length = (P_x-B_x)*(P_x-B_x) + (P_y-B_y)*(P_y-B_y);

        if(PA_square_length < 0.0001f || PB_square_length < 0.0001f){
            return false;
        }

        float PA_PB_cos = ( (P_x-A_x)*(P_x-B_x) + (P_y-B_y)*(P_y-A_y) ) / sqrt(PA_square_length) / sqrt(PB_square_length);

        if(fabs(PA_PB_cos) < 0.9f){
//            cout << "Not a line segment" <<endl;

            findNearestPoint2DOnALine(A_x, A_y, B_x, B_y, P_x, P_y, P_prime_x, P_prime_y);

            A_prime_x = A_x + (P_x - P_prime_x);
            A_prime_y = A_y + (P_y - P_prime_y);

            B_prime_x = B_x + (P_x - P_prime_x);
            B_prime_y = B_y + (P_y - P_prime_y);

        }else{ //the trajectory is a line segment

//            cout << "A line segment" <<endl;

            float norm_AB_x = B_x - A_x;
            float norm_AB_y = B_y - A_y;
            float norm_vertical_AB_1_x = norm_AB_y;
            float norm_vertical_AB_1_y = -norm_AB_x;
            float length = sqrt(norm_vertical_AB_1_x*norm_vertical_AB_1_x + norm_vertical_AB_1_y*norm_vertical_AB_1_y);
            norm_vertical_AB_1_x /= length;
            norm_vertical_AB_1_y /= length;

            float norm_vertical_AB_2_x = -norm_AB_y;
            float norm_vertical_AB_2_y = norm_AB_x;
            norm_vertical_AB_2_x /= length;
            norm_vertical_AB_2_y /= length;

            static const float width_half_for_line_envelope = 0.2f;
            A_prime_x = A_x + norm_vertical_AB_1_x * width_half_for_line_envelope;
            A_prime_y = A_y + norm_vertical_AB_1_y * width_half_for_line_envelope;

            B_prime_x = B_x + norm_vertical_AB_1_x * width_half_for_line_envelope;
            B_prime_y = B_y + norm_vertical_AB_1_y * width_half_for_line_envelope;

            A_x = A_x + norm_vertical_AB_2_x * width_half_for_line_envelope;
            A_y = A_y + norm_vertical_AB_2_y * width_half_for_line_envelope;

            B_x = B_x + norm_vertical_AB_2_x * width_half_for_line_envelope;
            B_y = B_y + norm_vertical_AB_2_y * width_half_for_line_envelope;

            P_x = (A_prime_x + B_prime_x) / 2.f;
            P_y = (A_prime_y + B_prime_y) / 2.f;

            P_prime_x = (A_x + B_x) / 2.f;
            P_prime_y = (A_y + B_y) / 2.f;
        }

//        if(fabs(A_x - B_x) + fabs(A_y - B_y) < 0.001f || fabs(A_prime_x - B_x) + fabs(A_prime_y - B_y) < 0.001f
//            || fabs(B_prime_x - B_x) + fabs(B_prime_y - B_y) < 0.01f || fabs(A_prime_x - A_x) + fabs(A_prime_y - A_y) < 0.01f){
//            cout <<"WTF!!!! A_x=" << A_x <<", A_y=" << A_y <<", B_x="<< B_x <<", B_y=" <<B_y << endl;
//            cout <<"A_prime_x=" << A_prime_x <<", A_prime_y=" << A_prime_y <<", B_prime_x="<< B_prime_x <<", B_prime_y=" <<B_prime_y << endl;
//            cout <<"P_x=" << P_x <<", P_y=" << P_y <<", P_prime_x="<< P_prime_x <<", P_prime_y=" <<P_prime_y << endl;
//            cout << "t=" <<t <<", vx="<< current_node->vx << ", ax="<<ax <<", vy="<< current_node->vy << ", ay="<<ay <<endl;
//        }



        /// Find min z and max z
        float z_potential_min_max_P_x = A_x;
        float z_potential_min_max_P_y = A_y;
        float z_potential_min_max_P_z = A_z;
        if(fabs(az) > 0.0001f){
            float t_vz_zero = -current_node->vz / az;
            if(t_vz_zero > 0.f && t_vz_zero < t)
            {
                z_potential_min_max_P_z = A_z + current_node->vz*t_vz_zero + 0.5f*az*t_vz_zero*t_vz_zero;
                z_potential_min_max_P_x = A_x + current_node->vx*t_vz_zero + 0.5f*ax*t_vz_zero*t_vz_zero;
                z_potential_min_max_P_y = A_y + current_node->vy*t_vz_zero + 0.5f*ay*t_vz_zero*t_vz_zero;
            }
        }

        Point3D z_min_point, z_max_point;
        z_min_point.x = z_potential_min_max_P_x;
        z_min_point.y = z_potential_min_max_P_y;
        z_min_point.z = z_potential_min_max_P_z;
        z_max_point.x = z_potential_min_max_P_x;
        z_max_point.y = z_potential_min_max_P_y;
        z_max_point.z = z_potential_min_max_P_z;

        if(A_z < z_min_point.z){
            z_min_point.x = A_x;
            z_min_point.y = A_y;
            z_min_point.z = A_z;
        }
        if(B_z < z_min_point.z){
            z_min_point.x = B_x;
            z_min_point.y = B_y;
            z_min_point.z = B_z;
        }

        if(A_z > z_max_point.z){
            z_max_point.x = A_x;
            z_max_point.y = A_y;
            z_max_point.z = A_z;
        }
        if(B_z > z_max_point.z){
            z_max_point.x = B_x;
            z_max_point.y = B_y;
            z_max_point.z = B_z;
        }

        if(z_max_point.z - z_min_point.z < 0.05f) // fix the bug when z_min_point.z == z_max_point.z
        {
            z_max_point.z += 0.05f;
            z_min_point.z -= 0.05f;
        }

        // Set surfaces
        Plane3D p1;
        p1.point.x = A_x; p1.point.y = A_y; p1.point.z = 0.f;
        p1.normal.x = P_prime_x - P_x; p1.normal.y = P_prime_y - P_y; p1.normal.z = 0.f;
        float norm_length1 = vectorNorm(p1.normal.x, p1.normal.y, p1.normal.z);
        p1.normal.x /= norm_length1;  p1.normal.y /= norm_length1; p1.normal.z /= norm_length1;
        envelope.surfaces.push_back(p1);

        Plane3D p2;
        p2.point.x = B_x; p2.point.y = B_y; p2.point.z = 0.f;
        p2.normal.x = B_x - A_x; p2.normal.y = B_y - A_y; p2.normal.z = 0.f;
        float norm_length2 = vectorNorm(p2.normal.x, p2.normal.y, p2.normal.z);
        p2.normal.x /= norm_length2;  p2.normal.y /= norm_length2; p2.normal.z /= norm_length2;
        envelope.surfaces.push_back(p2);

        Plane3D p3;
        p3.point.x = B_prime_x;  p3.point.y = B_prime_y; p3.point.z = 0.f;
        p3.normal.x = P_x - P_prime_x; p3.normal.y = P_y - P_prime_y; p3.normal.z = 0.f;
        float norm_length3 = vectorNorm(p3.normal.x, p3.normal.y, p3.normal.z);
        p3.normal.x /= norm_length3;  p3.normal.y /= norm_length3; p3.normal.z /= norm_length3;
        envelope.surfaces.push_back(p3);

        Plane3D p4;
        p4.point.x = A_prime_x;  p4.point.y = A_prime_y;  p4.point.z = 0.f;
        p4.normal.x = A_x - B_x; p4.normal.y = A_y - B_y; p4.normal.z= 0.f;
        float norm_length4 = vectorNorm(p4.normal.x, p4.normal.y, p4.normal.z);
        p4.normal.x /= norm_length4;  p4.normal.y /= norm_length4; p4.normal.z /= norm_length4;
        envelope.surfaces.push_back(p4);

        Plane3D p5;
        p5.point.x = z_min_point.x; p5.point.y = z_min_point.y; p5.point.z = z_min_point.z;
        p5.normal.x = 0.f; p5.normal.y = 0.f; p5.normal.z = -1.f;
        float norm_length5 = vectorNorm(p5.normal.x, p5.normal.y, p5.normal.z);
        p5.normal.x /= norm_length5;  p5.normal.y /= norm_length5; p5.normal.z /= norm_length5;
        envelope.surfaces.push_back(p5);

        Plane3D p6;
        p6.point.x = z_max_point.x; p6.point.y = z_max_point.y; p6.point.z = z_max_point.z;
        p6.normal.x = 0.f; p6.normal.y = 0.f; p6.normal.z = 1.f;
        float norm_length6 = vectorNorm(p6.normal.x, p6.normal.y, p6.normal.z);
        p6.normal.x /= norm_length6;  p6.normal.y /= norm_length6; p6.normal.z /= norm_length6;
        envelope.surfaces.push_back(p6);

        // Set vertexes
        Point3D v1, v2;
        v1.x = v2.x = A_x;  v1.y = v2.y = A_y; v1.z = z_max_point.z; v2.z = z_min_point.z;
        envelope.vertexes.push_back(v1);
        envelope.vertexes.push_back(v2);

        Point3D v3, v4;
        v3.x = v4.x = B_x;  v3.y = v4.y = B_y; v3.z = z_max_point.z; v4.z = z_min_point.z;
        envelope.vertexes.push_back(v3);
        envelope.vertexes.push_back(v4);

        Point3D v5, v6;
        v5.x = v6.x = B_prime_x;  v5.y = v6.y = B_prime_y; v5.z = z_max_point.z; v6.z = z_min_point.z;
        envelope.vertexes.push_back(v5);
        envelope.vertexes.push_back(v6);

        Point3D v7, v8;
        v7.x = v8.x = A_prime_x;  v7.y = v8.y = A_prime_y; v7.z = z_max_point.z; v8.z = z_min_point.z;
        envelope.vertexes.push_back(v7);
        envelope.vertexes.push_back(v8);

        return true;
    }


private:
    void nextStep(Node* current_node)
    {
        // Sample primitives
        for(auto const &ax : a_sample_vector_x){
            for(auto const &ay : a_sample_vector_y){
                for(auto const &az : a_sample_vector_z){

                    if(fabs(ax)+fabs(ay)+fabs(current_node->vx)+fabs(current_node->vy) < 0.01f)
                    {
                        // Avoid freezing
                        continue;
                    }

                    TrajPoint primitive_this_endpoint;
                    RectangleEnvelope envelope_this;

                    if(!getMotionPrimitive(current_node, ax, ay, az, primitive_this_endpoint, envelope_this))
                    {
                        continue;
                    }


                    static const float lambda = 0.01;
                    float weight = lambda*(ax*ax + ay*ay + az*az) + time_step_node; // To add on g
//                    float weight = time_step_node * 1.f;

                    // Consider direction change for the first step
                    if(current_node->time_stamp < 0.01f && fabs(reference_direction_angle) < M_PIf32){
                        weight += 0.1f * fabs(reference_direction_angle - atan2(primitive_this_endpoint.y - current_node->y,primitive_this_endpoint.x - current_node->x));
                    }

                    checkPoint(primitive_this_endpoint.x, primitive_this_endpoint.y, primitive_this_endpoint.z,
                               primitive_this_endpoint.vx, primitive_this_endpoint.vy, primitive_this_endpoint.vz,
                               current_node, weight, envelope_this);
                }
            }
        }

    }


    bool getMotionPrimitive(Node *current_node, float ax, float ay, float az, TrajPoint &primitive_end_point, RectangleEnvelope &envelope){

        primitive_end_point.vx = current_node->vx + ax * time_step_node;
        primitive_end_point.vy = current_node->vy + ay * time_step_node;
        primitive_end_point.vz = current_node->vz + az * time_step_node;
        primitive_end_point.x = current_node->x + current_node->vx*time_step_node + 0.5f*ax*time_step_node*time_step_node;
        primitive_end_point.y = current_node->y + current_node->vy*time_step_node + 0.5f*ay*time_step_node*time_step_node;
        primitive_end_point.z = current_node->z + current_node->vz*time_step_node + 0.5f*az*time_step_node*time_step_node;

        searched_point_vector.push_back(primitive_end_point);

        if(fabs(primitive_end_point.x - current_node->x) + fabs(primitive_end_point.y - current_node->y) + fabs(primitive_end_point.z - current_node->z)< 0.01){
            return false;
        }

        /// Calculate rectangle envelope
        RectangleEnvelope envelope_ori;
        if(!findRectangleEnvelope(current_node, ax, ay, az, time_step_node, envelope_ori)){
            return false;
        }

        /// Expand by a safety distance
        static const float half_safety_distance_for_z = safety_distance * 0.5f;
        static vector<float> primitive_envelope_expand_distance = {safety_distance, 0, safety_distance, safety_distance, half_safety_distance_for_z, half_safety_distance_for_z};
        expandEnvelope(envelope_ori, envelope, primitive_envelope_expand_distance); //VOXEL_RESOLUTION

        envelope.time_stamp_start = current_node->time_stamp;
        envelope.time_stamp_end = envelope.time_stamp_start + time_step_node;

        return true;
    }


    static void setMinAndMax(float &min, float &max, float current_value)
    {
        if(min > current_value){
            min = current_value;
        }
        if(max < current_value){
            max = current_value;
        }
    }


    void checkPoint(float x, float y, float z, float vx, float vy, float vz, Node* father,float g, RectangleEnvelope &envelope)
    {
        if (!checkNodeValid(x, y, z, vx, vy, vz))
            return;


//        std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
        if (!checkIfEnvelopeSafe(envelope, risk_limitation_motion_primitive, risk_limitation_single_voxel))
            return;
//        std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
//        std::cout << "Check: "
//                  << 1.0e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count()
//                  << "ms" << std::endl;

        /// Coding by voxel index and velocity direction.
//        static const int x_coding_step = MAP_WIDTH_VOXEL_NUM*MAP_HEIGHT_VOXEL_NUM*8;
//        static const int y_coding_step = MAP_HEIGHT_VOXEL_NUM*8;
//        static const int z_coding_step = 8;

        static const int x_coding_step = MAP_WIDTH_VOXEL_NUM*MAP_HEIGHT_VOXEL_NUM;
        static const int y_coding_step = MAP_HEIGHT_VOXEL_NUM;
        static const int z_coding_step = 1;

        long int coding_index = (int)((x + map_length_half) / VOXEL_RESOLUTION)* x_coding_step +
                              (int)((y + map_width_half) / VOXEL_RESOLUTION)* y_coding_step +
                              (int)((z + map_height_half) / VOXEL_RESOLUTION)* z_coding_step;
//        int v_direction_index = 0;
//        if(vx >= 0.f){
//            v_direction_index += 4;
//        }
//        if(vy >= 0.f){
//            v_direction_index += 2;
//        }
//        if(vz >= 0.f){
//            v_direction_index += 1;
//        }
//        coding_index += v_direction_index;

        /// Check if contained in close list
        if (ifContainedInVector(&close_list, x, y, z, vx, vy, vz) != -1)
            return;

        int index;
        if ((index = ifContainedInVector(&open_list, x, y, z, vx, vy, vz)) != -1)
        {
            Node *point = open_list[index];
            if (point->g > father->f + g)
            {
                point->father = father;
                point->g = father->g + g;
                point->f = point->g + point->h;
            }
        }
        else
        {
            Node * point = new Node(father->time_stamp + time_step_node, x,y,z,vx,vy,vz, father, envelope);
            point->coding_index = coding_index;

            calculateGHF(point, end_node, g);
            open_list.push_back(point);
        }
    }


    void addPath(Node *node){
        result_path.push_back(node);

        if (node->father != nullptr){
            addPath(node->father);
        }

    }

    void calculateGHF(Node* sNode, Node* eNode, float g)
    {
        // Estimated time
        float h = sqrt((sNode->x - eNode->x)*(sNode->x - eNode->x)  + (sNode->y - eNode->y)*(sNode->y - eNode->y) + (sNode->z - eNode->z)*(sNode->z - eNode->z)) / v_max_xy;

        float current_g = sNode->father->g + g;

        float f = current_g + h;
        sNode->f = f;
        sNode->h = h;
        sNode->g = current_g;
    }



    bool checkNodeValid(Node* node) const{
        if(node->x < -map_length_half  || node->x > map_length_half
          || node->y < -map_width_half  || node->y > map_width_half
          || node->z < -map_height_half  || node->z > map_height_half){
            return false;
        }

        if(fabs(node->vx) > v_max_xy || fabs(node->vy) > v_max_xy || fabs(node->vz) > v_max_z){
            return false;
        }

        return true;
    }


    bool checkNodeValid(float &x, float &y, float &z, float &vx, float &vy, float &vz) const{
        if(x < -map_length_half || x > map_length_half
           || y < -map_length_half || y > map_length_half
           || z < -map_length_half || z > map_length_half)
        {
            return false;
        }

        if(fabs(vx) > v_max_xy || fabs(vy) > v_max_xy || fabs(vz) > v_max_z){
            return false;
        }

        return true;
    }


    bool checkNodeOnBoundary(Node* node, float boundary_width = 1.0) const{
        if(node->x < -map_length_half + boundary_width || node->x >= map_length_half - boundary_width
           || node->y < -map_width_half + boundary_width || node->y >= map_width_half - boundary_width){ //|| node->z < -map_height_half + boundary_width || node->z >= map_height_half - boundary_width
            return true;
        }else{
            return false;
        }
    }


    bool checkIfEnvelopeSafe(RectangleEnvelope &envelope, float risk_threshold){
        vector<int> indexes;
        if(!getEnvelopeRectangleIndexes(envelope, indexes)){
            return false;
        }

        float risk_summation = 0.f;
        for(auto &index : indexes){
            float single_voxel_risk = *(risk_map + index);
            risk_summation += single_voxel_risk;
        }
        if(risk_summation > risk_threshold){
            return false;
        }else{
            return true;
        }
    }

    bool checkIfEnvelopeSafe(RectangleEnvelope &envelope, float risk_threshold, float risk_threshold_one_voxel){
        // height_max_limit
        if(use_height_limit){
            float height_max_limit_map = height_max_limit - map_center_z;
            float height_min_limit_map = height_min_limit - map_center_z;

            for(auto &p : envelope.vertexes){
                if(p.z > height_max_limit_map || p.z < height_min_limit_map){
                    return false;
                }
            }
        }

        // Get voxel storage indexes in the DSP map
        vector<int> indexes;
        if(!getEnvelopeRectangleIndexes(envelope, indexes)){
            return false;
        }

        float risk_summation = 0.f;
        for(auto &index : indexes){
            float single_voxel_risk = *(risk_map + index);
            if(single_voxel_risk > risk_threshold_one_voxel){
                return false;
            }else{
                risk_summation += single_voxel_risk;
                if(risk_summation > risk_threshold){
                    return false;
                }
            }
        }

        return true;
    }

    void matchTimeToTimeIndex(int &time_stamp_indexes_to_check, float time_stamp_start)
    {
        /// Correct the time with planning start time
        time_stamp_start += start_time;
        time_stamp_indexes_to_check = floor(time_stamp_start/time_step_node);
        if(time_stamp_indexes_to_check >= RISK_MAP_NUMBER){
            time_stamp_indexes_to_check = RISK_MAP_NUMBER - 1;
        }
    }


    bool getEnvelopeRectangleIndexes(RectangleEnvelope &envelope, vector<int> &indexes){ // indexes: dimension 1: position, dimension 2: time
        /// Match time
        int time_stamp_index_to_check;
        matchTimeToTimeIndex(time_stamp_index_to_check, envelope.time_stamp_start);

        /// Check in a large rectangle and get indexes
        float x_min, x_max, y_min, y_max, z_min, z_max;
        x_min = x_max = envelope.vertexes[0].x;
        y_min = y_max = envelope.vertexes[0].y;
        z_min = z_max = envelope.vertexes[0].z;
        for(int i=1; i<8; ++i){
            if(x_min > envelope.vertexes[i].x){
                x_min = envelope.vertexes[i].x;
            }
            if(y_min > envelope.vertexes[i].y){
                y_min = envelope.vertexes[i].y;
            }
            if(z_min > envelope.vertexes[i].z){
                z_min = envelope.vertexes[i].z;
            }

            if(x_max < envelope.vertexes[i].x){
                x_max = envelope.vertexes[i].x;
            }
            if(y_max < envelope.vertexes[i].y){
                y_max = envelope.vertexes[i].y;
            }
            if(z_max < envelope.vertexes[i].z){
                z_max = envelope.vertexes[i].z;
            }
        }

        int x_voxels_num = ceil((x_max - x_min) / VOXEL_RESOLUTION);
        int y_voxels_num = ceil((y_max - y_min) / VOXEL_RESOLUTION);
        int z_voxels_num = ceil((z_max - z_min) / VOXEL_RESOLUTION);

        auto x_index_start = (int)((x_min + map_length_half) / VOXEL_RESOLUTION);
        auto y_index_start = (int)((y_min + map_width_half) / VOXEL_RESOLUTION);
        auto z_index_start = (int)((z_min + map_height_half) / VOXEL_RESOLUTION);

        static const int z_change_storage_taken = MAP_WIDTH_VOXEL_NUM*MAP_LENGTH_VOXEL_NUM*RISK_MAP_NUMBER;  //Order: zyxt
        static const int y_change_storage_taken = MAP_LENGTH_VOXEL_NUM*RISK_MAP_NUMBER;
        static const int x_change_storage_taken = RISK_MAP_NUMBER;

//        static const int check_step = 1;

        for(int i=0; i<=x_voxels_num; ++i){
            auto x_index_this = x_index_start + i;
            if(x_index_this < 0 || x_index_this >= MAP_LENGTH_VOXEL_NUM){
                return false;
            }

            for(int j=0; j<=y_voxels_num; ++j){
                auto y_index_this = y_index_start + j;
                if(y_index_this < 0 || y_index_this >= MAP_WIDTH_VOXEL_NUM){
                    return false;
                }

                for(int k=0; k<z_voxels_num; ++k){
                    auto z_index_this = z_index_start + k;
                    if(z_index_this < 0 || z_index_this >= MAP_HEIGHT_VOXEL_NUM){
                        return false;
                    }

                    /// Ignore the grids that are not inside of the envelope
                    float grid_center_x = x_min + VOXEL_RESOLUTION*i;
                    float grid_center_y = y_min + VOXEL_RESOLUTION*j;
                    float grid_center_z = z_min + VOXEL_RESOLUTION*k;
                    if(!ifPointInEnvelope(grid_center_x, grid_center_y, grid_center_z, envelope)){
                        continue;
                    }

                    int index = z_index_this * z_change_storage_taken + y_index_this * y_change_storage_taken + x_index_this*x_change_storage_taken + time_stamp_index_to_check;
                    indexes.push_back(index);
                }
            }
        }
        return true;
    }


//    bool checkIfFreeSpace(int &check_index){
//
//        if(*(risk_map+check_index) > 0.05){
//            return false;
//        }
//
//        return true;
//    }


    int ifContainedInVector(vector<Node*>* Nodelist, float &x, float &y, float &z,
                                   float &vx, float &vy, float &vz) const
    {
        /// This function is hard to tune!!!!
        for(int i = 0; i < Nodelist->size(); i++)
        {
            float same_node_threshold = VOXEL_RESOLUTION * std::min((fabs(vx)+fabs(vy)+fabs(vz)+0.0001f)*time_step_node, 1.f);

            if (fabs(Nodelist->at(i)->x - x) < same_node_threshold &&
                fabs(Nodelist->at(i)->y - y) < same_node_threshold &&
                fabs(Nodelist->at(i)->z - z) < same_node_threshold //&&
                )
            {
                return i;
            }
        }
        return -1;
    }


    int ifContainedInVector(vector<Node*>* Nodelist, int coding_index) const
    {
        for(int i = 0; i < Nodelist->size(); i++)
        {
            if (Nodelist->at(i)->coding_index == coding_index){
                return i;
            }
        }
        return -1;
    }


    static bool fourDIndexToOneDIndex(const int &x_index, const int &y_index, const int &z_index, const int time_index, int &index){
        /// Get Index
        static const int z_change_storage_taken = MAP_WIDTH_VOXEL_NUM*MAP_LENGTH_VOXEL_NUM*RISK_MAP_NUMBER;
        static const int y_change_storage_taken = MAP_LENGTH_VOXEL_NUM*RISK_MAP_NUMBER;
        static const int x_change_storage_taken = RISK_MAP_NUMBER;

        index = z_index * z_change_storage_taken + y_index * y_change_storage_taken + x_index*x_change_storage_taken + time_index;

        if(index < 0 || index >= VOXEL_NUM*RISK_MAP_NUMBER){
            return false;
        }else{
            return true;
        }
    }


    static bool compare(Node* n1,Node* n2){
        return n1->f < n2->f;
    }


    static bool nodeEqual(Node* n1, Node* n2, float threshold = 1.f){
        if(fabs(n1->x - n2->x) < threshold && fabs(n1->y - n2->y) < threshold && fabs(n1->z - n2->z) < threshold){
            return true;
        }else{
            return false;
        }
    }


public:
    vector<Node*> result_path_reversed;

    float time_step_node;
    float time_step_trajectory;

    float v_max_xy;
    float v_max_z;

    float a_min_x;
    float a_min_y;
    float a_max_xy;
    float a_sample_step_xy;

    float a_min_z;
    float a_max_z;
    float a_sample_step_z;

    float boundary_width;

    float risk_limitation_motion_primitive;
    float risk_limitation_corridor;
    float risk_limitation_single_voxel;

private:
    vector<Node*> open_list;
    vector<Node*> close_list;
    vector<Node*> result_path;
    Node *start_node{};
    Node *end_node{};

    float start_time;

    float map_length_half;
    float map_width_half;
    float map_height_half;

    float height_max_limit;
    float height_min_limit;
    bool use_height_limit;
    bool sample_z_acc;

    float map_center_x;
    float map_center_y;
    float map_center_z;
    float safety_distance;
    float reference_direction_angle;

    vector<float> a_sample_vector_x;
    vector<float> a_sample_vector_y;
    vector<float> a_sample_vector_z;

    vector<TrajPoint> searched_point_vector;

 float *risk_map{};
};

