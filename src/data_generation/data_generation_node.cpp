#include "path_planner/path_planner.h"
#include <chrono>

using Clock = std::chrono::high_resolution_clock;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataset_generator");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");
    Visualization visualization(nh, "prm_visual");
    
    // ===== 地图列表 =====
    std::vector<std::string> map_list = {
        "/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/octo_binary/octomap.bt",
        "/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/octo_binary/octomap.bt"
    };


    auto ms = std::make_shared<mapping_simulator::MappingSimulator>();


    for (const auto& map_file : map_list)
    {
        ms->resetMap(map_file);

        PathPlanner planner(ms, true, 1.0);

        for (int i = 0; i < 10; ++i)
        {
            double sx, sy, sz, gx, gy, gz;

            planner.sampleValidStartGoal(sx, sy, sz, gx, gy, gz);

            planner.setStart(sx, sy, sz);
            planner.setGoal(gx, gy, gz);

            if (!planner.solve(2))
            {
                ROS_WARN("Plan failed");
                continue;
            }

            // =========================
            // ✅ 1. 可视化 PRM + 路径
            // =========================

            auto path = planner.extractSolutionPath();
            visualization.setPathData(path);

            auto [nodes, edges] = planner.extractGraph();
            visualization.setGraphData(nodes, edges);

            visualization.visualizeStartAndGoal(
                sx, sy, sz,
                gx, gy, gz
            );

            // 🔥 很关键（刷新RViz）
            ros::spinOnce();

            // =========================
            // ✅ 2. 数据采样
            // =========================

            auto sampled = planner.resamplePath(path, 0.1);

            ROS_INFO("Path length: %zu", sampled.size());

            for (size_t k = 0; k < sampled.size(); ++k)
            {
                Eigen::Vector3d pose = sampled[k];

                // ===== yaw =====
                double yaw = 0.0;
                if (k < sampled.size() - 1)
                {
                    Eigen::Vector3d dir = sampled[k+1] - sampled[k];
                    yaw = std::atan2(dir.y(), dir.x());
                }

                // ===== pitch / roll =====
                double pitch = 30.0 * M_PI / 180.0;
                double roll  = 0.0;

                Eigen::Matrix3d R =
                    (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();



                    auto t1 = Clock::now();

                    pcl::PointCloud<pcl::PointXYZ> lidar_pointcloud;
                    ms->simulateLidar(pose, R, lidar_pointcloud);

                    auto t2 = Clock::now();

                    // ===== Local Map =====
                    pcl::PointCloud<pcl::PointXYZ> occ_pointcloud, free_pointcloud;
                    ms->extractLocalMap(pose, occ_pointcloud, free_pointcloud);

                    auto t3 = Clock::now();


                    // ===== 打印时间 =====
                    double lidar_time =
                    std::chrono::duration<double, std::milli>(t2 - t1).count();
                    double map_time =
                    std::chrono::duration<double, std::milli>(t3 - t2).count();

                    // ROS_INFO("Time: lidar=%.2f ms, local_map=%.2f ms",
                    //         lidar_time, map_time);

                    // ROS_INFO("Sample: lidar=%zu, occ=%zu",
                    //     lidar_pointcloud.size(),
                    //     occ_pointcloud.size());

                ros::spinOnce();
            }
        }
    }

    return 0;
}