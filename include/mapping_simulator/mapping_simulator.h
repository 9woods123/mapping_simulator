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

namespace mapping_simulator{

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

    void extractLocalMap(const Eigen::Vector3d& center, pcl::PointCloud<pcl::PointXYZ> & local_occ,
                                        pcl::PointCloud<pcl::PointXYZ> & local_free);



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

};
}

#endif // ESDF_MAP_GENERATOR_H
