# ORB-SLAM3-ROS with Ceres Loop Optimization

A ROS implementation of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 that focuses on the ROS part, with additional Ceres-based loop optimization pipeline.
This package uses ```catkin build```. Tested on Ubuntu 20.04.

## Project Structure
```
├── orb_slam3_ros/          # Main ORB-SLAM3 ROS package
├── 3DCeres/                # Ceres optimization module
│   └── loop_optimization.cpp
├── data/                   # Generated optimization data
└── docker/                 # Docker configuration files
```

## 1. Environment Setup

### Option A: Native Ubuntu Installation

#### System Requirements
- Ubuntu 20.04 LTS
- ROS Noetic
- GCC 7.5+ or Clang 6+
- CMake 3.16+

#### Prerequisites Installation

##### Eigen3
```bash
sudo apt update
sudo apt install libeigen3-dev
```

##### Pangolin
```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

##### Ceres Solver
```bash
# Install dependencies
sudo apt install libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev

# Build Ceres from source
cd ~
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

##### OpenCV (if not already installed)
```bash
sudo apt install libopencv-dev python3-opencv
```

##### (Optional) `hector-trajectory-server`
Install `hector-trajectory-server` to visualize the real-time trajectory of the camera/imu.
```bash
sudo apt install ros-noetic-hector-trajectory-server
```

### Option B: Docker Installation

#### Build Docker Environment
```bash
# Clone the repository
git clone https://github.com/your-repo/orb_slam3_ros_ceres.git
cd orb_slam3_ros_ceres

# Build Docker image
docker build -t orb_slam3_ceres:latest -f docker/Dockerfile .
```

#### Docker Compose Setup
```bash
# Start the complete environment
docker-compose up -d

# Enter the container
docker exec -it orb_slam3_container bash
```

#### Dockerfile Content
The Docker environment includes:
- Ubuntu 20.04 base image
- ROS Noetic full installation
- All required dependencies (Eigen3, Pangolin, Ceres, OpenCV)
- Pre-configured workspace with proper permissions

## 2. Installation

### Clone and Build the Workspace
```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/thien94/orb_slam3_ros.git
git clone https://github.com/your-repo/3DCeres.git

# Build the workspace
cd ~/catkin_ws
catkin build

# Source the workspace
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Build Ceres Optimization Module
```bash
cd ~/catkin_ws/src/3DCeres
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 3. Usage Pipeline

### Step 1: Run ORB-SLAM3 to Generate Optimization Data

#### Prepare Dataset
For KITTI dataset example:
```bash
# Download KITTI dataset
mkdir -p ~/datasets/kitti
cd ~/datasets/kitti
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
unzip 2011_09_26_drive_0002_sync.zip

# Convert to ROS bag (optional, use existing tools)
# Follow: https://stevenliu216.github.io/2018/08/05/working-with-kitti-ros/
```

#### Launch ORB-SLAM3
```bash
# Terminal 1: Launch ORB-SLAM3 with data export enabled
roslaunch orb_slam3_ros kitti_stereo.launch enable_data_export:=true export_path:=~/catkin_ws/data

# Terminal 2: Play dataset
rosbag play kitti_2011_09_26_drive_0002_synced.bag
```

#### Alternative: Real-time Stereo Camera
```bash
# For ZED camera
roslaunch orb_slam3_ros zed_stereo.launch

# For Intel RealSense
roslaunch orb_slam3_ros realsense_stereo.launch
```

### Step 2: Extract Optimization Data

After running ORB-SLAM3, the following data will be automatically exported to `~/catkin_ws/data/`:
- `keyframe_poses.txt`: Keyframe poses
- `map_points.txt`: 3D map points
- `loop_closures.txt`: Detected loop closure constraints
- `pose_graph.txt`: Pose graph edges
- `trajectory.txt`: Camera trajectory

#### Manual Data Export (if needed)
```bash
# Save trajectory and map
rosservice call /orb_slam3/save_traj session_data
rosservice call /orb_slam3/save_map session_map

# Export additional optimization data
rosservice call /orb_slam3/export_optimization_data ~/catkin_ws/data/optimization_data
```

### Step 3: Run Ceres Loop Optimization

#### Basic Optimization
```bash
cd ~/catkin_ws/src/3DCeres/build

# Run loop optimization with default parameters
./loop_optimization \
    --input_path ~/catkin_ws/data \
    --output_path ~/catkin_ws/results \
    --max_iterations 100 \
    --robust_kernel huber
```

#### Advanced Optimization with Custom Parameters
```bash
# Run with custom optimization settings
./loop_optimization \
    --input_path ~/catkin_ws/data \
    --output_path ~/catkin_ws/results \
    --max_iterations 200 \
    --robust_kernel cauchy \
    --robust_kernel_scale 1.0 \
    --pose_weight 1.0 \
    --point_weight 0.1 \
    --loop_weight 10.0 \
    --verbose
```

#### Batch Processing Multiple Sessions
```bash
# Process multiple datasets
for dataset in session_01 session_02 session_03; do
    ./loop_optimization \
        --input_path ~/catkin_ws/data/$dataset \
        --output_path ~/catkin_ws/results/$dataset \
        --config_file config/optimization_config.yaml
done
```

## 4. Configuration

### ORB-SLAM3 Settings
Edit the settings file (e.g., `config/KITTI04-12.yaml`) to enable data export:
```yaml
# Data Export Settings
System.ExportOptimizationData: 1
System.ExportPath: "/home/user/catkin_ws/data"
System.ExportInterval: 10  # Export every 10 keyframes
```

### Ceres Optimization Parameters
Create `config/optimization_config.yaml`:
```yaml
optimization:
  max_iterations: 100
  robust_kernel: "huber"
  robust_kernel_scale: 1.0
  
weights:
  pose_prior: 1.0
  landmark_observation: 0.1
  loop_closure: 10.0
  
convergence:
  function_tolerance: 1e-6
  gradient_tolerance: 1e-10
  parameter_tolerance: 1e-8
```

## 5. Output Analysis

### Optimization Results
After running Ceres optimization, check the results:
```bash
# View optimization statistics
cat ~/catkin_ws/results/optimization_summary.txt

# Compare before/after trajectories
python3 scripts/compare_trajectories.py \
    --original ~/catkin_ws/data/trajectory.txt \
    --optimized ~/catkin_ws/results/optimized_trajectory.txt \
    --output ~/catkin_ws/results/comparison_plot.png
```

### Visualization
```bash
# Launch RViz with optimized trajectory
roslaunch orb_slam3_ros visualize_results.launch \
    optimized_trajectory:=~/catkin_ws/results/optimized_trajectory.txt

# Generate 3D visualization
cd ~/catkin_ws/src/3DCeres/build
./visualize_optimization \
    --result_path ~/catkin_ws/results \
    --output_format ply
```

## 6. ROS Topics, Params and Services

### Subscribed Topics
- `/camera/image_raw` for Mono(-Inertial) node
- `/camera/left/image_raw` for Stereo(-Inertial) node  
- `/camera/right/image_raw` for Stereo(-Inertial) node
- `/imu` for Mono/Stereo/RGBD-Inertial node
- `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` for RGBD node

### Published Topics
- `/orb_slam3/camera_pose`: left camera pose in world frame, published at camera rate
- `/orb_slam3/body_odom`: imu-body odometry in world frame, published at camera rate
- `/orb_slam3/tracking_image`: processed image from the left camera with key points and status text
- `/orb_slam3/tracked_points`: all key points contained in the sliding window
- `/orb_slam3/all_points`: all key points in the map
- `/orb_slam3/kf_markers`: markers for all keyframes' positions
- `/orb_slam3/optimization_data`: real-time optimization data for Ceres processing
- `/tf`: with camera and imu-body poses in world frame

### Params
- `voc_file`: path to vocabulary file required by ORB-SLAM3
- `settings_file`: path to settings file required by ORB-SLAM3
- `enable_pangoli
