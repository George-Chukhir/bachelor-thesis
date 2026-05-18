#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::string pkg_dir = "/home/stringer/b2_ws/src/b2_thesis_fusion/maps";
    std::string pcd_file = pkg_dir + "/fastlio2_maps/map.pcd";
    std::string map_name = pkg_dir + "/2d_maps/2d_map";
    
    double resolution = 0.05; // 5 cm per pixel
    double z_min = 0.2;
    double z_max = 1.0;

    std::cout << "1. Loading " << pcd_file << "...\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
        std::cerr << "Error: Couldn't read file " << pcd_file << "\n";
        return -1;
    }

    std::cout << "2. Height slice (Z from " << z_min << "m to " << z_max << "m)...\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_filtered);

    if (cloud_filtered->empty()) {
        std::cerr << "Error: Filtered cloud is empty! Check your Z limits or point cloud.\n";
        return -1;
    }

    std::cout << "3. Projection on 2D grid...\n";
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& pt : cloud_filtered->points) {
        if (pt.x < min_x) min_x = pt.x;
        if (pt.x > max_x) max_x = pt.x;
        if (pt.y < min_y) min_y = pt.y;
        if (pt.y > max_y) max_y = pt.y;
    }

    int width = static_cast<int>(std::ceil((max_x - min_x) / resolution));
    int height = static_cast<int>(std::ceil((max_y - min_y) / resolution));

    // 254 - free space, 0 - occupied space
    cv::Mat grid(height, width, CV_8UC1, cv::Scalar(254));

    for (const auto& pt : cloud_filtered->points) {
        int idx_x = static_cast<int>((pt.x - min_x) / resolution);
        int idx_y = static_cast<int>((pt.y - min_y) / resolution);
        
        // Boundaries check
        if (idx_x >= 0 && idx_x < width && idx_y >= 0 && idx_y < height) {
            // Solve mismatch between ROS origin (bottom-left) and OpenCV origin (top-left)
            // Invert Y for correct display
            int idx_y_img = height - 1 - idx_y;
            grid.at<uchar>(idx_y_img, idx_x) = 0; // mark obstacle
        }
    }

    // Thicken walls by 1 pixel (so AMCL and laser rays can better latch onto them)
    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
    cv::erode(grid, grid, kernel, cv::Point(-1, -1), 1);

    std::cout << "4. Saving PGM and YAML...\n";
    std::string pgm_path = map_name + ".pgm";
    cv::imwrite(pgm_path, grid);

    YAML::Emitter out;
    out << YAML::BeginMap;
    // Just save the basename for the image, as normally done in ROS
    out << YAML::Key << "image" << YAML::Value << "2d_map.pgm"; 
    out << YAML::Key << "resolution" << YAML::Value << resolution;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << min_x << min_y << 0.0 << YAML::EndSeq;
    out << YAML::Key << "negate" << YAML::Value << 0;
    out << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    out << YAML::Key << "free_thresh" << YAML::Value << 0.196;
    out << YAML::EndMap;

    std::string yaml_path = map_name + ".yaml";
    std::ofstream fout(yaml_path);
    fout << out.c_str();
    fout.close();

    std::cout << "Success! Files " << pgm_path << " and " << yaml_path << " created\n";
    return 0;
}
