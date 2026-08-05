整理成 README 风格如下，保持你的原始信息，同时把数据生成流程、参数说明、代码位置整理清楚。

```markdown
# Data Generation Pipeline


This module generates training data for 3D scene completion and planning.

The data generation process uses **RRT-STAR (RRT*)** to generate random collision-free trajectories in the environments.

Along each generated path, we simulate LiDAR measurements and extract local map observations from the global map.


## Overview


Given:

```

INPUT:
- Global point cloud / Octomap map
- Sensor pose trajectory

OUTPUT:
- Simulated LiDAR measurements
- Local ground-truth map patches

```


The generated data contains:

- LiDAR point clouds from simulated sensor observations
- Sensor poses
- Local voxel maps cropped from the global ground truth map


---

# 1. Data Generation Pipeline


The whole pipeline is:


```

Global Map (Octomap)
|
|
v
RRT* Path Planning
|
|
v
Random Collision-Free Path
|
|
v
Simulate LiDAR Scan
|
|
v
Extract Local Map Ground Truth
|
|
v
Training Data

````


---

# 2. Generate Dataset


## Build ROS Environment


```bash
source devel/setup.bash
````

Launch data generation:

```bash
roslaunch mapping_simulator data_generation.launch
```

---

# 3. Generate Data on Multiple Maps

The default map list is defined in:

```
mapping_simulator/src/data_generation/data_generation_node.cpp
```

Find:

```cpp
// ===== 地图列表 =====
std::vector<std::string> map_list = {

    "/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/octo_binary/octomap_forest_val.bt",

};
```

Modify this list to generate data on different environments.

Example:

```cpp
std::vector<std::string> map_list = {

    "octomap_forest_val.bt",

    "octomap_random.bt",

    "octomap_office.bt"

};
```

Maps are stored in:

```
mapping_simulator/octo_binary/
```

The `.bt` files are generated from Unreal Engine (UE).

---

# 4. LiDAR Simulation and Local Map Parameters

The main parameters are located at:

```
mapping_simulator/mapping_simulator.cpp
```

## 4.1 LiDAR Resolution

```cpp
nh_private_.param("hrz_lines",
                  hrz_lines_,
                  120);


nh_private_.param("vtc_lines",
                  vtc_lines_,
                  16);


nh_private_.param("vtc_fov_deg",
                  vtc_fov_deg,
                  60.0);
```

Parameters:

| Parameter   | Meaning                        | Default |
| ----------- | ------------------------------ | ------- |
| hrz_lines   | Horizontal scanning resolution | 120     |
| vtc_lines   | Vertical scanning resolution   | 16      |
| vtc_fov_deg | Vertical field of view         | 60°     |

Example:

```
16 vertical beams
60 degree vertical FOV
```

---

## 4.2 Local Map Parameters

```cpp
nh_private_.param("max_range",
                  max_range_,
                  10.0);


nh_private_.param("local_map_size_x",
                  local_map_size_x,
                  16.0);


nh_private_.param("local_map_size_y",
                  local_map_size_y,
                  16.0);


nh_private_.param("local_map_size_z",
                  local_map_size_z,
                  8.0);
```

Parameters:

| Parameter        | Meaning                |
| ---------------- | ---------------------- |
| max_range        | LiDAR sensing range    |
| local_map_size_x | Local map size along X |
| local_map_size_y | Local map size along Y |
| local_map_size_z | Local map size along Z |

### LiDAR Observation Range

`max_range_` defines the LiDAR simulation range:
The simulated LiDAR points are generated around the current sensor pose.
---

### Local Ground Truth Map

`local_map_size_x/y/z` defines the local ground truth map volume:

Example:

```
local map:

        Z
        |
        |
        +------Y
       /
      X


16m × 16m × 8m
```

This local map is used as the ground truth for scene completion.

---

# 5. LiDAR Sensor Tilt Configuration

The LiDAR mounting angle is defined in:

```
mapping_simulator/src/data_generation/data_generation_node.cpp
```

Current configuration:

```cpp
// ===== pitch / roll =====

double pitch = 30.0 * M_PI / 180.0;

double roll  = 0.0;


// ===== Construct Rotation Matrix =====

Eigen::Matrix3d R =
    (Eigen::AngleAxisd(
        yaw,
        Eigen::Vector3d::UnitZ()
     )
     *
     Eigen::AngleAxisd(
        pitch,
        Eigen::Vector3d::UnitY()
     )
     *
     Eigen::AngleAxisd(
        roll,
        Eigen::Vector3d::UnitX()
     ))
    .toRotationMatrix();
```

The sensor orientation is:

```
Yaw:
    Robot heading direction


Pitch:
    LiDAR downward/upward tilt


Roll:
    Sensor rotation around forward axis
```

Current setting:

| Parameter | Value                |
| --------- | -------------------- |
| Pitch     | 30°                  |
| Roll      | 0°                   |
| Yaw       | trajectory dependent |

---

# 6. Generated Data Description


The generated dataset format is defined by the data saving function:

mapping_simulator/include/data_generation/data_generation.h

```

mapping_simulator/include/data_generation/data_generation.h

````


The data structure is implemented in:


```cpp
class DataSaver {

public:

    /**
     * Save generated data into TXT files
     */
    static void SaveData2txtfile(
        ...
    );

};
````

The exact saved fields and file organization are determined by:

```cpp
DataSaver::SaveData2txtfile()
```

---

# 7. Notes

1. The global map must be converted into Octomap `.bt` format before simulation.

2. Different environments can be added by modifying:

```
data_generation_node.cpp
```

3. Sensor characteristics can be changed by modifying:

```
mapping_simulator.cpp
```

4. LiDAR installation angle can be modified in:

```
data_generation_node.cpp
```

5. Changing `max_range_` affects LiDAR observation area, while changing `local_map_size_*` affects the ground-truth supervision region.

```




