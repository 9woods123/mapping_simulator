# load_simple_txt.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
# 在文件开头添加字体设置

# 设置中文字体
# 方法1: 使用系统自带的字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 支持中文的字体
plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号

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
    map_res = list(map(float, lines[idx].split()))
    idx += 1
    data['map_resolution'] = map_res[0] if len(map_res) > 0 else 0.0

    lidar_range = list(map(float, lines[idx].split()))
    idx += 1
    data['lidar_max_range'] = lidar_range[0] if len(lidar_range) > 0 else 0.0
    
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

def visualize_all_pointclouds(data, save_path=None):
    """
    可视化所有点云
    """
    # 创建一个大图
    fig = plt.figure(figsize=(20, 12))
    
    # 1. 激光雷达点云
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    if len(data['lidar_points']) > 0:
        ax1.scatter(data['lidar_points'][:, 0], 
                   data['lidar_points'][:, 1], 
                   data['lidar_points'][:, 2],
                   c='blue', s=1, alpha=0.6, label=f'LiDAR ({len(data["lidar_points"])}点)')
    ax1.set_title('激光雷达点云')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()
    
    # 2. 建图障碍点
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    if len(data['occ_points']) > 0:
        ax2.scatter(data['occ_points'][:, 0], 
                   data['occ_points'][:, 1], 
                   data['occ_points'][:, 2],
                   c='red', s=2, alpha=0.8, label=f'障碍物 ({len(data["occ_points"])}体素)')
    ax2.set_title('建图障碍点')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()
    
    # 3. 建图自由点
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    if len(data['free_points']) > 0:
        ax3.scatter(data['free_points'][:, 0], 
                   data['free_points'][:, 1], 
                   data['free_points'][:, 2],
                   c='green', s=1, alpha=0.6, label=f'自由空间 ({len(data["free_points"])}体素)')
    ax3.set_title('建图自由点')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.legend()
    
    # 4. 真值障碍点
    ax4 = fig.add_subplot(2, 3, 4, projection='3d')
    if len(data['occ_points_gt']) > 0:
        ax4.scatter(data['occ_points_gt'][:, 0], 
                   data['occ_points_gt'][:, 1], 
                   data['occ_points_gt'][:, 2],
                   c='red', s=2, alpha=0.8, label=f'障碍物GT ({len(data["occ_points_gt"])}体素)')
    ax4.set_title('真值障碍点')
    ax4.set_xlabel('X')
    ax4.set_ylabel('Y')
    ax4.set_zlabel('Z')
    ax4.legend()
    
    # 5. 真值自由点
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    if len(data['free_points_gt']) > 0:
        ax5.scatter(data['free_points_gt'][:, 0], 
                   data['free_points_gt'][:, 1], 
                   data['free_points_gt'][:, 2],
                   c='green', s=1, alpha=0.6, label=f'自由空间GT ({len(data["free_points_gt"])}体素)')
    ax5.set_title('真值自由点')
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_zlabel('Z')
    ax5.legend()
    
    # 6. 所有点云叠加对比
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')
    
    # 绘制位姿
    pose = data['pose']
    ax6.scatter([pose[0]], [pose[1]], [pose[2]], 
               c='orange', s=100, marker='*', label='传感器位姿')
    
    # 绘制LiDAR点云
    if len(data['lidar_points']) > 0:
        ax6.scatter(data['lidar_points'][:, 0], 
                   data['lidar_points'][:, 1], 
                   data['lidar_points'][:, 2],
                   c='blue', s=1, alpha=0.3, label=f'LiDAR')
    
    # 绘制建图障碍点
    if len(data['occ_points']) > 0:
        ax6.scatter(data['occ_points'][:, 0], 
                   data['occ_points'][:, 1], 
                   data['occ_points'][:, 2],
                   c='red', s=2, alpha=0.5, label=f'建图障碍')
    
    # 绘制建图自由点
    if len(data['free_points']) > 0:
        ax6.scatter(data['free_points'][:, 0], 
                   data['free_points'][:, 1], 
                   data['free_points'][:, 2],
                   c='green', s=1, alpha=0.2, label=f'建图自由')
    
    ax6.set_title('所有点云叠加')
    ax6.set_xlabel('X')
    ax6.set_ylabel('Y')
    ax6.set_zlabel('Z')
    ax6.legend(loc='upper right', fontsize='small')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"图片已保存: {save_path}")
    
    plt.show()

def visualize_occupancy_comparison(data, save_path=None):
    """
    可视化建图结果与真值的对比
    """
    fig = plt.figure(figsize=(15, 6))
    
    # 1. 建图结果
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    
    # 绘制障碍物
    if len(data['occ_points']) > 0:
        ax1.scatter(data['occ_points'][:, 0], 
                   data['occ_points'][:, 1], 
                   data['occ_points'][:, 2],
                   c='red', s=5, alpha=0.8, label=f'障碍物 ({len(data["occ_points"])})')
    
    # 绘制自由空间
    if len(data['free_points']) > 0:
        ax1.scatter(data['free_points'][:, 0], 
                   data['free_points'][:, 1], 
                   data['free_points'][:, 2],
                   c='green', s=1, alpha=0.3, label=f'自由空间 ({len(data["free_points"])})')
    
    # 绘制传感器位姿
    pose = data['pose']
    ax1.scatter([pose[0]], [pose[1]], [pose[2]], 
               c='orange', s=100, marker='*', label='传感器位姿')
    
    ax1.set_title('建图结果')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()
    
    # 2. 真值地图
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    
    # 绘制障碍物真值
    if len(data['occ_points_gt']) > 0:
        ax2.scatter(data['occ_points_gt'][:, 0], 
                   data['occ_points_gt'][:, 1], 
                   data['occ_points_gt'][:, 2],
                   c='red', s=5, alpha=0.8, label=f'障碍物GT ({len(data["occ_points_gt"])})')
    
    # 绘制自由空间真值
    if len(data['free_points_gt']) > 0:
        ax2.scatter(data['free_points_gt'][:, 0], 
                   data['free_points_gt'][:, 1], 
                   data['free_points_gt'][:, 2],
                   c='green', s=1, alpha=0.3, label=f'自由空间GT ({len(data["free_points_gt"])})')
    
    ax2.set_title('真值地图')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()
    
    # 3. 对比
    ax3 = fig.add_subplot(1, 3, 3, projection='3d')
    
    # 绘制建图障碍物
    if len(data['occ_points']) > 0:
        ax3.scatter(data['occ_points'][:, 0], 
                   data['occ_points'][:, 1], 
                   data['occ_points'][:, 2],
                   c='red', s=3, alpha=0.5, label='建图障碍物')
    
    # 绘制真值障碍物
    if len(data['occ_points_gt']) > 0:
        ax3.scatter(data['occ_points_gt'][:, 0], 
                   data['occ_points_gt'][:, 1], 
                   data['occ_points_gt'][:, 2],
                   c='blue', s=2, alpha=0.7, marker='^', label='真值障碍物')
    
    # 绘制传感器位姿
    ax3.scatter([pose[0]], [pose[1]], [pose[2]], 
               c='orange', s=100, marker='*', label='传感器位姿')
    
    ax3.set_title('建图 vs 真值对比')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.legend()
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"对比图已保存: {save_path}")
    
    plt.show()

def print_statistics(data):
    """打印数据统计信息"""
    print("=" * 60)
    print("数据统计信息:")
    print("-" * 60)
    print(f"位姿: {data['pose']}")
    print(f"地图分辨率: {data['map_resolution']}")
    print(f"激光雷达量程: {data['lidar_max_range']}")
    print(f"边界框: X[{data['min_x']:.2f}, {data['max_x']:.2f}], "
          f"Y[{data['min_y']:.2f}, {data['max_y']:.2f}], "
          f"Z[{data['min_z']:.2f}, {data['max_z']:.2f}]")
    print(f"点云数量:")
    print(f"  激光雷达点云: {len(data['lidar_points'])}")
    print(f"  建图障碍点: {len(data['occ_points'])}")
    print(f"  建图自由点: {len(data['free_points'])}")
    print(f"  真值障碍点: {len(data['occ_points_gt'])}")
    print(f"  真值自由点: {len(data['free_points_gt'])}")
    
    # 计算覆盖率
    if len(data['occ_points_gt']) + len(data['free_points_gt']) > 0:
        total_gt = len(data['occ_points_gt']) + len(data['free_points_gt'])
        total_mapped = len(data['occ_points']) + len(data['free_points'])
        coverage = total_mapped / total_gt * 100 if total_gt > 0 else 0
        print(f"建图覆盖率: {coverage:.1f}% ({total_mapped}/{total_gt})")
    print("=" * 60)

# 使用示例
if __name__ == "__main__":
    filename = "/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/dataset/sample_1.txt"
    
    try:
        # 加载数据
        data = load_sample_txt(filename)
        print(f"成功加载: {filename}")
        print(f"位姿: {data['pose']}")
        
        # 打印统计信息
        print_statistics(data)
        
        # 可视化所有点云
        visualize_all_pointclouds(data, save_path="all_pointclouds.png")
        
        # 可视化对比
        visualize_occupancy_comparison(data, save_path="occupancy_comparison.png")
        
    except Exception as e:
        print(f"加载或可视化失败: {e}")
        import traceback
        traceback.print_exc()