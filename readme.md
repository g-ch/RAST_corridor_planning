
# Compile
To compile the map. You need to install PCL and munkers-cpp (https://github.com/saebyn/munkres-cpp). 

# Basic Usage
An example of using this map is presented in map_sim_example.cpp

**The input is point cloud. 

**The output is current occupancy status and future occupancy status. Use function "getOccupancyMapWithFutureStatus" to get the output.

**Currently, we publish the current occupancy status with topic "/my_map/cloud_ob" and one layer of the predicted future occupancy maps with topic "/my_map/future_status".

# Parameters
Skip the following if you use default parameters. Other wise find them in dsp-nongaussian-dst-new.h \
1. Change the range and resolution of the map by changing the following parameters:
```
#define MAP_LENGTH_VOXEL_NUM 66 //33//29  //odd
#define MAP_WIDTH_VOXEL_NUM 66 //33//29   //odd
#define MAP_HEIGHT_VOXEL_NUM 41 //9 //21, 9 //odd
#define VOXEL_RESOLUTION 0.15
#define ANGLE_RESOLUTION 3
#define MAX_PARTICLE_NUM_VOXEL 30 
```
**With different resolutions, you also need to change the threshold in "getOccupancyMapWithFutureStatus".

2. Set the camera FOV angle for a new camera:
```
const int half_fov_h = 45;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a little smaller value than the real FOV angle
const int half_fov_v = 27;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a little smaller value than the real FOV angle
```

3. Change the time stamp to predict future status
```
#define PREDICTION_TIMES 6
static const float prediction_future_time[PREDICTION_TIMES] = {0.05f, 0.2f, 0.5f, 1.f, 1.5f, 2.f}; //unit: second
```




