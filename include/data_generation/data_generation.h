// simple_data_saver.hpp
#ifndef SIMPLE_DATA_SAVER_H
#define SIMPLE_DATA_SAVER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>

class DataSaver {
public:
    /**
     * 保存数据到TXT文件
     */
    static void SaveData2txtfile(
        const std::string& output_dir,
        int sample_id,
        const Eigen::Vector3d& pose,
        const Eigen::Matrix3d& R,
        double map_resolution,
        double lidar_max_range,
        double map_min_x, double map_min_y,
        double map_min_z, double map_max_x,
        double map_max_y, double map_max_z,
        double local_map_size_x, double local_map_size_y, double local_map_size_z,
        const pcl::PointCloud<pcl::PointXYZ>& lidar_pointcloud,
        const pcl::PointCloud<pcl::PointXYZ>& occ_pointcloud,
        const pcl::PointCloud<pcl::PointXYZ>& free_pointcloud,
        const pcl::PointCloud<pcl::PointXYZ>& occ_pointcloud_gt,
        const pcl::PointCloud<pcl::PointXYZ>& free_pointcloud_gt
    ) {
        // 1. 创建目录
        createDirectory(output_dir);
        
        // 2. 生成文件名
        std::string filename = output_dir + "/sample_" + 
                              std::to_string(sample_id) + ".txt";
        
        // 3. 打开文件
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << filename << std::endl;
            return;
        }
        
        // 设置浮点数精度
        file << std::setprecision(6) << std::fixed;
        
        // ========== 1. 头部信息 ==========
        // 位姿
        file << pose.x() << " " << pose.y() << " " << pose.z() << "\n";
        
        // 旋转矩阵
        file << R(0,0) << " " << R(0,1) << " " << R(0,2) << "\n";
        file << R(1,0) << " " << R(1,1) << " " << R(1,2) << "\n";
        file << R(2,0) << " " << R(2,1) << " " << R(2,2) << "\n";
        
        // 参数
        file << map_resolution << "\n";
        
        file <<lidar_max_range << "\n";
        
        // 地图边界框
        file << map_min_x << " " << map_max_x << "\n";
        file << map_min_y << " " << map_max_y << "\n";
        file << map_min_z << " " << map_max_z << "\n";
        
        // 局部地图边界框
        file << local_map_size_x << " " << local_map_size_y <<local_map_size_z<< "\n";

        // ========== 2. 点云数量 ==========
        file << lidar_pointcloud.size() << "\n";
        file << occ_pointcloud.size() << "\n";
        file << free_pointcloud.size() << "\n";
        file << occ_pointcloud_gt.size() << "\n";
        file << free_pointcloud_gt.size() << "\n";
        
        // ========== 3. 点云数据 ==========
        // 激光雷达点云
        for (const auto& p : lidar_pointcloud) {
            file << p.x << " " << p.y << " " << p.z << "\n";
        }
        
        // 建图障碍点
        for (const auto& p : occ_pointcloud) {
            file << p.x << " " << p.y << " " << p.z << "\n";
        }
        
        // 建图自由点
        for (const auto& p : free_pointcloud) {
            file << p.x << " " << p.y << " " << p.z << "\n";
        }
        
        // 真值障碍点
        for (const auto& p : occ_pointcloud_gt) {
            file << p.x << " " << p.y << " " << p.z << "\n";
        }
        
        // 真值自由点
        for (const auto& p : free_pointcloud_gt) {
            file << p.x << " " << p.y << " " << p.z << "\n";
        }
        
        file.close();
        
        // std::cout << "Saved sample " << sample_id << " to: " << filename << std::endl;
    }
    
private:
    // 创建目录
    static void createDirectory(const std::string& path) {
        std::string cmd = "mkdir -p " + path;
        system(cmd.c_str());
    }
};

#endif // SIMPLE_DATA_SAVER_H
