#ifndef MAPPING_SIMULATOR_H
#define MAPPING_SIMULATOR_H

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <string>       
#include <unordered_set>

namespace mapping_simulator{


// struct VoxelKey {
//     int x, y, z;
    
//     VoxelKey(int ix, int iy, int iz) : x(ix), y(iy), z(iz) {}
    
//     bool operator==(const VoxelKey& other) const {
//         return x == other.x && y == other.y && z == other.z;
//     }
    
//     std::string toString() const {
//         return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z);
//     }
// };

// // 哈希函数
// struct VoxelKeyHash {
//     std::size_t operator()(const VoxelKey& k) const {
//         // 简单哈希
//         return ((k.x * 73856093) ^ (k.y * 19349663) ^ (k.z * 83492791));
//     }
// };


class MappingSimulator {
public:
    MappingSimulator();
    ~MappingSimulator();
    
    void loadOctomap();
    void generateESDF();
    void publishMaps();
    void publishCallback(const ros::TimerEvent&);
    void simdata_pubCallback(const ros::TimerEvent&);

    void convertOctomapToRosMsg();
    void convertEsdfToPointCloudMsg();
    bool getMinCollisionDistanceAndGradient(float x, float y, float z, 
    float& min_distance, Eigen::Vector3f& gradient);
    bool isPointOccupied(double x, double y, double z);
    bool isPointOccupiedWithVolume(double x, double y, double z, double radius);
    void getMapBounds(double &min_x, double &min_y, double &min_z,double &max_x, double &max_y, double &max_z);
    

    void resetMap(std::string octomap_file_);

    void simulateLidar(const Eigen::Vector3d& origin,const Eigen::Matrix3d& R,pcl::PointCloud<pcl::PointXYZ>& lidar_pointcloud );

    // void extractLocalMap(const Eigen::Vector3d& center, pcl::PointCloud<pcl::PointXYZ> & local_occ,
    //                                     pcl::PointCloud<pcl::PointXYZ> & local_free);

    void extractLocalMap(const Eigen::Vector3d& center, const pcl::PointCloud<pcl::PointXYZ>& lidar_pointcloud,
                        pcl::PointCloud<pcl::PointXYZ> & local_occ, pcl::PointCloud<pcl::PointXYZ> & local_free,
                        pcl::PointCloud<pcl::PointXYZ> & local_occ_gt, pcl::PointCloud<pcl::PointXYZ> & local_free_gt);
    
    void simulateMappingFromPointCloud(
            const Eigen::Vector3d& sensor_origin,
            const pcl::PointCloud<pcl::PointXYZ>& lidar_points,
            pcl::PointCloud<pcl::PointXYZ>& local_occ,
            pcl::PointCloud<pcl::PointXYZ>& local_free,
            const octomap::point3d& min_bound,
            const octomap::point3d& max_bound,
            double voxel_size);

    void getLocalMapSize(double &local_x_size, double &local_y_size, double &local_z_size);
    double getMapResolution();
    double getLidarMaxRange();
private:
    struct VoxelID {
        int x, y, z;
        bool operator==(const VoxelID& other) const { return x == other.x && y == other.y && z == other.z; }
    };

    struct VoxelIDHasher {
        size_t operator()(const VoxelID& voxel) const {
            return (static_cast<size_t>(voxel.x) << 32) ^ (static_cast<size_t>(voxel.y) << 16) ^ static_cast<size_t>(voxel.z);
        }
    };

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher octomap_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher gt_pointcloud_pub_;


    ros::Publisher esdf_pub_;
    ros::Timer timer_; // 定时器
    ros::Timer simdata_pub_timer_; // 定时器

    std::string octomap_file_;
    double resolution_;

    octomap::OcTree tree_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr octo_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr freespace_cloud_;
    std::unordered_map<VoxelID, float, VoxelIDHasher> esdf_map_;

    // ROS 消息
    octomap_msgs::Octomap octomap_msg_;                // 用于存储 OctoMap 的 ROS 消息
    sensor_msgs::PointCloud2 esdf_msg_;               // 用于存储 ESDF 的 ROS 点云消息
    
    sensor_msgs::PointCloud2 local_map_occ_msg_;
    sensor_msgs::PointCloud2 local_map_occ_gt_msg_;
    sensor_msgs::PointCloud2 lidar_pointcloud_msg_;



    // lidar simulator params
    ros::Publisher lidar_pub_;

    int hrz_lines_;
    int vtc_lines_;
    double max_range_;
    double vtc_fov_rad_;
    double vtc_fov_deg;


    // ground truth local map
    double local_map_size_x;
    double local_map_size_y;
    double local_map_size_z;

    //====================================================================
    // 坐标到体素索引

    inline Eigen::Vector3i coordToVoxelIndex(
        double x, double y, double z,
        const octomap::point3d& min_bound,
        double voxel_size) const {
        return Eigen::Vector3i(
            static_cast<int>((x - min_bound.x()) / voxel_size),
            static_cast<int>((y - min_bound.y()) / voxel_size),
            static_cast<int>((z - min_bound.z()) / voxel_size)
        );
    }
    
    // 体素索引到体素中心坐标
    inline octomap::point3d voxelIndexToCenter(
        const Eigen::Vector3i& index,
        const octomap::point3d& min_bound,
        double voxel_size) const {
        return octomap::point3d(
            min_bound.x() + (index.x() + 0.5) * voxel_size,
            min_bound.y() + (index.y() + 0.5) * voxel_size,
            min_bound.z() + (index.z() + 0.5) * voxel_size
        );
    }


};

}
#endif // ESDF_MAP_GENERATOR_H
