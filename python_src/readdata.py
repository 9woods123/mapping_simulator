# load_simple_txt.py
import numpy as np

def load_sample_txt(filename):
    """加载TXT格式的样本"""
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    data = {}
    idx = 0
    
    # 1. 位姿
    pose = list(map(float, lines[idx].split()))
    data['pose'] = np.array(pose)
    idx += 1
    
    # 2. 旋转矩阵
    R = []
    for _ in range(3):
        row = list(map(float, lines[idx].split()))
        R.append(row)
        idx += 1
    data['R'] = np.array(R)
    
    # 3. 参数
    map_res = map(float, lines[idx].split())
    idx += 1
    data['map_resolution'] = map_res

    lidar_range = map(float, lines[idx].split())
    idx += 1
    data['lidar_max_range'] = lidar_range
    
    # 4. 边界框
    data['min_x'], data['max_x'] = map(float, lines[idx].split())
    idx += 1
    data['min_y'], data['max_y'] = map(float, lines[idx].split())
    idx += 1
    data['min_z'], data['max_z'] = map(float, lines[idx].split())
    idx += 1
    
    # 5. 点云数量
    lidar_points_counts = list(map(int, lines[idx].split()))
    idx += 1

    occ_points_counts = list(map(int, lines[idx].split()))
    idx += 1
    
    free_points_counts = list(map(int, lines[idx].split()))
    idx += 1
    
    occ_points_gt_counts = list(map(int, lines[idx].split()))
    idx += 1

    free_points_gt_counts = list(map(int, lines[idx].split()))
    idx += 1
    

    print("lidar_points_counts:",)
    # 6. 点云数据
    data['lidar_points'] = np.array([list(map(float, lines[idx+i].split())) 
                                     for i in range(lidar_points_counts[0])])
    idx += lidar_points_counts[0]
    
    data['occ_points'] = np.array([list(map(float, lines[idx+i].split())) 
                                   for i in range(occ_points_counts[0])])
    idx += occ_points_counts[0]
    
    data['free_points'] = np.array([list(map(float, lines[idx+i].split())) 
                                    for i in range(free_points_counts[0])])
    idx += free_points_counts[0]
    
    data['occ_points_gt'] = np.array([list(map(float, lines[idx+i].split())) 
                                      for i in range(occ_points_gt_counts[0])])
    idx += occ_points_gt_counts[0]
    
    data['free_points_gt'] = np.array([list(map(float, lines[idx+i].split())) 
                                       for i in range(free_points_gt_counts[0])])
    
    return data

# 使用示例
if __name__ == "__main__":
    data = load_sample_txt("/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/dataset/sample_1.txt")
    print(f"位姿: {data['pose']}")

    # print("data['free_points_gt'][-1]:",data['free_points_gt'][-2])


# 0571 8820 6060 一般排2-3年。正式入学后