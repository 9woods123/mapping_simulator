#include "path_planner/rrtstar.h"
#include "path_planner/visualization.h"
#include <chrono>

using Clock = std::chrono::high_resolution_clock;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataset_generator");



    ros::NodeHandle nh;
    Visualization visualization(nh, "rrtstar_vis");

    // ===== 地图列表 =====
    std::vector<std::string> map_list = {
        "/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/octo_binary/octomap.bt",
        "/home/easy/easy_ws/zju_phd_ws/ensemble_aware_planning_ws/ros_ws/src/mapping_simulator/octo_binary/octomap.bt"
    };

    auto ms = std::make_shared<mapping_simulator::MappingSimulator>();

    for (const auto& map_file : map_list)
    {
        ms->resetMap(map_file);

        RRTStarPlanner planner(ms, true, 0.75);


        for (int i = 0; i < 100; ++i)
        {

            // double sx, sy, sz, gx, gy, gz;
            double sx = -10, sy = -10, sz = 1.5;
            double gx =  10, gy = 10, gz = 2.0;
            // ===== 随机 start / goal =====
            planner.sampleValidStartGoal(sx, sy, sz, gx, gy, gz, 10.0);
            

            planner.setStart(sx, sy, sz);
            planner.setGoal(gx, gy, gz);
            
            
            bool success=planner.solve(8.0, 1.0 , 1.0);



            if (success)
            {
                ROS_INFO("RRT* success!");

                // ===== 1. path =====
                auto path_nodes = planner.getSolutionPath();

                visualization.setPathData(path_nodes);

                auto [nodes, edges] = planner.getGraph();
                visualization.setGraphData(nodes, edges);

                visualization.visualizeStartAndGoal(sx, sy, sz, gx, gy, gz);

                ros::spinOnce();


                // =========================
                // ✅ 2. 转成 Eigen path
                // =========================
                std::vector<Eigen::Vector3d> path;
                for (auto& n : path_nodes)
                {
                    path.emplace_back(n->pos);
                }

                // =========================
                // ✅ 4. 遍历轨迹 → 生成数据
                // =========================
                for (size_t k = 0; k < path.size(); ++k)
                {
                    Eigen::Vector3d pose = path[k];

                    // ===== yaw（沿路径方向）=====
                    double yaw=0;
                    if (k < path.size() - 1)
                    {
                        Eigen::Vector3d dir = path[k+1] - path[k];
                        yaw = std::atan2(dir.y(), dir.x());
                    }
                    else 
                    {   //最后一个航向角和倒数第二个一致
                        Eigen::Vector3d dir = path[k] - path[k-1];
                        yaw = std::atan2(dir.y(), dir.x());
                    }


                    // ===== pitch / roll =====
                    double pitch = 30.0 * M_PI / 180.0;
                    double roll  = 0.0;

                    // ===== 构造 R =====
                    Eigen::Matrix3d R =
                        (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();

                    // =========================
                    // ✅ 5. Lidar（计时）
                    // =========================
                    auto t1 = Clock::now();

                    pcl::PointCloud<pcl::PointXYZ> lidar_pointcloud;
                    ms->simulateLidar(pose, R, lidar_pointcloud);

                    auto t2 = Clock::now();

                    // =========================
                    // ✅ 6. Local Map（计时）
                    // =========================
                    pcl::PointCloud<pcl::PointXYZ> occ_pointcloud, free_pointcloud;
                    ms->extractLocalMap(pose, occ_pointcloud, free_pointcloud);

                    auto t3 = Clock::now();

                    double lidar_time =
                        std::chrono::duration<double, std::milli>(t2 - t1).count();

                    double map_time =
                        std::chrono::duration<double, std::milli>(t3 - t2).count();

                    // ROS_INFO("Time: lidar=%.2f ms, map=%.2f ms | pts: %zu",
                    //         lidar_time, map_time, lidar_pointcloud.size());

                    ros::spinOnce();
                    // ros::Duration(0.5).sleep();

                }
            }
            else
            {
                ROS_WARN("RRT* failed");
            }

        }
    }

    return 0;
}