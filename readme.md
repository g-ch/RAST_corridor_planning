
# Introduction
This package contains the code to build the risk-aware spatio-temporal (RAST) safety corridors and plan a minimum snap trajectory for MAV obstacle avoidance in dynamic uncertain environments. In the ``src`` folder, ``map_sim_example.cpp`` is the node that builds the DSP map and publishes risks in point cloud form. ``planning_node.cpp`` is the node that receives the calculated risks, generates RAST safety corridors, and plans minimum snap trajectory. The interfaces are designed for a PX4 MAV.


# Compile
__Tested environment__: Ubuntu 18.04 + ROS Melodic and Ubuntu 20.04 + ROS Noetic

To compile the source code, you need:
1. PCL, Mavros, Eigen. PCL and Eigen are included in the desktop-full version of ROS. Mavros is only used for ROS message subscriptions in the example node. Check [mavros](https://github.com/mavlink/mavros) for installation guidance.

2. Install [munkers-cpp](https://github.com/saebyn/munkres-cpp) with the following steps.
    ```
    git clone https://github.com/saebyn/munkres-cpp.git
    cd munkres-cpp
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```
3. Install [OSQP](https://github.com/osqp/osqp), which is a lightweight QP solver. You can follow these [installation guidelines](https://osqp.org/docs/get_started/sources.html#build-the-binaries).
    ```
    git clone --recursive https://github.com/osqp/osqp
    cd osqp
    mkdir build && cd build
    cmake -G "Unix Makefiles" ..
    cmake --build .
    sudo cmake --build . --target install
    ```

4. Clone the code in a ROS workspace, update the submodule, and compile.
   ```
   mkdir -p rast_ws/src
   cd rast_ws/src
   git clone https://github.com/g-ch/RAST_corridor_planning.git
   cd RAST_corridor_planning
   git submodule init & git submodule update
   cd ../..
   catkin_make
   ```


# Basic Usage
## Input and output Details
The pipeline is __Environment -> Mapping (include risk calculation) -> Planning (include RAST corridor building) -> Trajectory tracking__.

### Mapping
Necessary input to build the DSP map and calculate risk with the ``map_sim_example`` mapping node are:
1) Point cloud from a depth camera in topic "/camera_front/depth/points" with msg type ``sensor_msgs::PointCloud2``. 
2) MAV position data from Mavros in topic "/mavros/local_position/pose" with msg type ``geometry_msgs::PoseStamped``.

__Note:__ Building the DSP map requires the FOV parameter of the depth camera. See [DSP Map](https://github.com/g-ch/DSP-map).

### Planning
The ``planning_node`` uses the output risk from ``map_sim_example`` and MAV position and velocity data 
1) MAV position data from Mavros in topic "/mavros/local_position/pose" with msg type ``geometry_msgs::PoseStamped``.
2) MAV velocity data from Mavros in topic "/mavros/local_position/velocity_local" with msg type ``geometry_msgs::TwistStamped``.

The output trajectory commands include two forms:
 1) Topic "/pva_setpoint" with msg type ``trajectory_msgs::JointTrajectoryPoint``, which contains one position (x,y,z,yaw), velocity and acceleration target. In our experiments, we use this output form and our [pva_tracker](https://github.com/g-ch/pva_tracker) to control a PX4 drone. 
 2) Topic "/command/trajectory" with msg type ``trajectory_msgs::MultiDOFJointTrajectory``, which contains the position (x,y,z), velocity and acceleration command of 20 steps, where the first step is the current state of the MAV. This output form is designed for mpc-based trackers. If the planned trajectory is less than 20 steps, we use a constant velocity model to predict the rest steps. One step is 0.05s in our code.

## Test in Simulation
### Quick Test with a ROS Bag
Download a bag file named `street.bag` containing the point cloud and pose data collected with a MAV in Gazebo. [Download](https://drive.google.com/file/d/1go4ALTe8CqaBY2wjZJzkUCmdlBI7yAAU/view?usp=sharing).
Save the bag file in the data folder and launch a test by
```
roslaunch rast_corridor_planning quick_test.launch
```

### Test in Gazebo Simulation Environment
We use the PX4 + Gazebo simulation environment. Details of the simulation environment can be found at [PX4+Gazebo](https://docs.px4.io/master/en/simulation/gazebo.html). After you have installed the simulation environment, you should modify the variable ``sdf`` in ``posix_sitl.launch`` to use a MAV with a depth camera. In our tests, we disabled the left and the right camera in ``iris_triple_depth_camera.sdf`` and use it in ``posix_sitl.launch``. 

Launch a simulation test.
1) Start the simulation environment by opening a command window in your PX4 source code main folder and run:
   ```
    DONT_RUN=1 make px4_sitl_default gazebo
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
    roslaunch px4 posix_sitl.launch 
   ```

2) Start Mavros:
    ```
    roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
    ```
3) Hover the drone and start the tracker. You can do this step by using our  
[pva_tracker](https://github.com/g-ch/pva_tracker).
    ```
    rosrun pva_tracker tracker_sim_auto_arm_takeoff
    ```

4) Start mapping and planning by
   ```
   roslaunch rast_corridor_planning planning.launch
   ```

The goal position and parameters related to the planning can be changed in file ``cfg/cfg.yaml``.

## Test with a Physical MAV
It is recommended to use a PX4 MAV with a realsense depth camera and NUC or Xavier NX computing board. The pipeline is the same as in the simulation. 


# Liciense
MIT Liciense.

# Additional Information
For more Information abou the DSP map, please refer to [DSP Map](https://github.com/g-ch/DSP-map) and [preprint](https://arxiv.org/abs/2202.06273).


