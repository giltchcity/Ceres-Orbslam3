# ORB-SLAM3-ROS with Ceres Loop Optimization

A ROS implementation of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 with Ceres-based loop optimization pipeline.

## Quick Start

### 1. Docker Environment Setup

#### Check and Start Container
```bash
# Check container status
docker ps -a

# Start container if exited
docker start orbslam3

# Enter container
docker exec -it orbslam3 bash
```

### 2. Modify ORB-SLAM3 for Data Export

#### Edit Loop Closing Files
```bash
# Edit header file
mousepad /Datasets/catkin_ws/src/orb_slam3_ros/orb_slam3/include/LoopClosing.h

# Edit source file  
mousepad /Datasets/catkin_ws/src/orb_slam3_ros/orb_slam3/src/LoopClosing.cc
```

#### Build the Project
```bash
cd /Datasets/catkin_ws/
catkin_make -DCMAKE_CXX_FLAGS="-w"
```

### 3. Run ORB-SLAM3 to Generate Data

#### Terminal 1: Launch ORB-SLAM3
```bash
roslaunch orb_slam3_ros kitti_stereo.launch
```

#### Terminal 2: Play Dataset
```bash
rosbag play /Datasets/KITTI/kitti_2011_09_30_drive_0020_synced.bag
```

### 4. Run Ceres Optimization

#### Clean and Build Ceres Program
```bash
cd /Datasets/CERES_Work/build
rm -rf *
cd /Datasets/CERES_Work/
rm ceres_example
make clean
make
```

#### Run Optimization
```bash
./ceres_example
```

### 5. Visualize Results

```bash
cd /home/jixian/Desktop/orbslam3_docker/Datasets/CERES_Work/Vis_Result/
python3 vis3.py
```

## Project Structure

```
├── orb_slam3_ros/              # Main ORB-SLAM3 ROS package
├── CERES_Work/                 # Ceres optimization module
│   ├── build/                  # Build directory
│   ├── ceres_example           # Main optimization executable
│   └── Vis_Result/             # Visualization scripts
│       └── vis3.py
└── KITTI/                      # Dataset directory
    └── kitti_2011_09_30_drive_0020_synced.bag
```

## Alternative Launch Options

### For Kimera Thoth Dataset
```bash
# Use provided kimera configuration
roslaunch orb_slam3_ros kimera_thoth.launch bag_file:=/path/to/thoth_dataset.bag
```

### For kitti 
```bash
# Remove bag file argument for live camera feed
roslaunch orb_slam3_ros kitti_stereo.launch
```

## Data Flow

1. **ORB-SLAM3** runs and generates optimization data during loop closure detection
2. **Ceres program** reads the exported data and performs bundle adjustment
3. **Visualization script** displays the optimized trajectory and map points

## Files Included

- `launch/kimera_thoth.launch` - Launch file for Kimera Thoth dataset  
- `config/RGB-D-Inertial/kimera_thoth.yaml` - Camera parameters for Thoth
- `3DCeres/loop_optimization.cpp` - Main Ceres optimization code
- `Vis_Result/vis3.py` - Results visualization script

## Troubleshooting

### Container Issues
```bash
# If container won't start
docker restart orbslam3

# Check container logs  
docker logs orbslam3
```

### Build Issues
```bash
# Clean workspace completely
cd /Datasets/catkin_ws/
rm -rf build/ devel/
catkin_make -DCMAKE_CXX_FLAGS="-w"
```

### Ceres Build Issues
```bash
# Complete clean rebuild
cd /Datasets/CERES_Work/
make clean
rm -rf build/*
make
```

## Expected Output

After running the complete pipeline:
- ORB-SLAM3 trajectory data saved in workspace
- Optimized trajectory from Ceres solver  
- 3D visualization showing before/after comparison
- Loop closure constraints and optimized p
