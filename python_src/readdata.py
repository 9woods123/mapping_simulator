# load_simple_txt.py
import numpy as np

def load_sample_txt(filename):

    # // 头部
    # pose.x() pose.y() pose.z()           // 1行: 位姿
    # R[0,0] R[0,1] R[0,2]                // 2行: 旋转矩阵第1行
    # R[1,0] R[1,1] R[1,2]                // 3行: 旋转矩阵第2行
    # R[2,0] R[2,1] R[2,2]                // 4行: 旋转矩阵第3行
    # map_resolution                      // 5行: 地图分辨率
    # lidar_max_range                     // 6行: 激光雷达量程
    # map_min_x map_max_x                 // 7行: 地图X边界
    # map_min_y map_max_y                 // 8行: 地图Y边界
    # map_min_z map_max_z                 // 9行: 地图Z边界
    # local_map_size_x local_map_size_y local_map_size_z  // 10行: 局部地图尺寸
    # lidar_pointcloud.size()             // 11行: LiDAR点数
    # occ_pointcloud.size()               // 12行: 建图障碍点数
    # free_pointcloud.size()              // 13行: 建图自由点数
    # occ_pointcloud_gt.size()            // 14行: 真值障碍点数
    # free_pointcloud_gt.size()           // 15行: 真值自由点数
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    data = {}
    idx = 0
    
    try:
        # 1. 位姿 (1行3个值)
        pose = list(map(float, lines[idx].split()))
        data['pose'] = np.array(pose[:3])
        idx += 1
        
        # 2. 旋转矩阵 (3行，每行3个值)
        R = []
        for _ in range(3):
            row = list(map(float, lines[idx].split()))
            R.append(row[:3])  # 只取前3个
            idx += 1
        data['R'] = np.array(R)
        
        # 3. 参数 (2行，每行1个值)
        data['map_resolution'] = float(lines[idx].split()[0])
        idx += 1
        
        data['lidar_max_range'] = float(lines[idx].split()[0])
        idx += 1
        
        # 4. 全局地图边界 (3行，每行2个值)
        data['map_min_x'], data['map_max_x'] = map(float, lines[idx].split()[:2])
        idx += 1
        
        data['map_min_y'], data['map_max_y'] = map(float, lines[idx].split()[:2])
        idx += 1
        
        data['map_min_z'], data['map_max_z'] = map(float, lines[idx].split()[:2])
        idx += 1
        
        # 5. 局部地图尺寸 (1行3个值) - 新增
        local_sizes = list(map(float, lines[idx].split()[:3]))
        data['local_map_size_x'] = local_sizes[0] if len(local_sizes) > 0 else 0.0
        data['local_map_size_y'] = local_sizes[1] if len(local_sizes) > 1 else 0.0
        data['local_map_size_z'] = local_sizes[2] if len(local_sizes) > 2 else 0.0
        idx += 1
        
        # 6. 点云数量 (5行，每行1个值)
        counts = []
        for _ in range(5):  # 读取5行
            if idx < len(lines):
                count = int(lines[idx].split()[0]) if lines[idx].strip() else 0
                counts.append(count)
            else:
                counts.append(0)
            idx += 1
        
        # 打印调试信息
        print(f"点云数量: LiDAR={counts[0]}, Occ={counts[1]}, Free={counts[2]}, "
              f"Occ_GT={counts[3]}, Free_GT={counts[4]}")
        
        # 7. 点云数据
        # LiDAR点云
        if counts[0] > 0:
            lidar_points = []
            for i in range(counts[0]):
                if idx + i < len(lines):
                    parts = lines[idx + i].split()
                    if len(parts) >= 3:
                        lidar_points.append(list(map(float, parts[:3])))
            data['lidar_points'] = np.array(lidar_points)
        else:
            data['lidar_points'] = np.array([])
        idx += counts[0]
        
        # 建图障碍点
        if counts[1] > 0:
            occ_points = []
            for i in range(counts[1]):
                if idx + i < len(lines):
                    parts = lines[idx + i].split()
                    if len(parts) >= 3:
                        occ_points.append(list(map(float, parts[:3])))
            data['occ_points'] = np.array(occ_points)
        else:
            data['occ_points'] = np.array([])
        idx += counts[1]
        
        # 建图自由点
        if counts[2] > 0:
            free_points = []
            for i in range(counts[2]):
                if idx + i < len(lines):
                    parts = lines[idx + i].split()
                    if len(parts) >= 3:
                        free_points.append(list(map(float, parts[:3])))
            data['free_points'] = np.array(free_points)
        else:
            data['free_points'] = np.array([])
        idx += counts[2]
        
        # 真值障碍点
        if counts[3] > 0:
            occ_points_gt = []
            for i in range(counts[3]):
                if idx + i < len(lines):
                    parts = lines[idx + i].split()
                    if len(parts) >= 3:
                        occ_points_gt.append(list(map(float, parts[:3])))
            data['occ_points_gt'] = np.array(occ_points_gt)
        else:
            data['occ_points_gt'] = np.array([])
        idx += counts[3]
        
        # 真值自由点
        if counts[4] > 0:
            free_points_gt = []
            for i in range(counts[4]):
                if idx + i < len(lines):
                    parts = lines[idx + i].split()
                    if len(parts) >= 3:
                        free_points_gt.append(list(map(float, parts[:3])))
            data['free_points_gt'] = np.array(free_points_gt)
        else:
            data['free_points_gt'] = np.array([])
        
        return data
        
    except Exception as e:
        print(f"加载文件 {filename} 时出错，行 {idx}: {e}")
        print(f"行内容: {lines[idx] if idx < len(lines) else 'EOF'}")
        raise


# 使用示例
if __name__ == "__main__":
    data = load_sample_txt("/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/dataset/sample_1.txt")
    print(f"位姿: {data['pose']}")

    print("data['free_points_gt'][-1]:",data['free_points_gt'][-2])


# 0571 8820 6060 一般排2-3年。正式入学后