#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>

#include <cstdint>
#include <string>
#include <vector>

namespace open3d_loc
{

struct RelocalizerOptions
{
    Eigen::Vector3d submap_size = Eigen::Vector3d(30.0, 30.0, 8.0);
    Eigen::Vector3d submap_stride = Eigen::Vector3d(15.0, 15.0, 4.0);
    int min_submap_points = 500;
    double voxel_size = 0.4;
    int scan_min_points = 300;
    int top_k = 5;
    double nearby_radius = 30.0;
    bool enable_global_search = true;
    double ransac_min_fitness = 0.15;
    double ransac_max_rmse = 2.0;
    double ransac_distance_multiplier = 2.0;
    bool enable_diagnostics = true;
    int diagnostics_top_n = 5;
    double icp_min_fitness = 0.45;
    double icp_max_rmse = 0.8;
    double max_pose_jump = 20.0;
    int descriptor_ring_count = 20;
    int descriptor_sector_count = 60;
    double descriptor_max_radius = 25.0;
    int descriptor_top_k = 12;
    double descriptor_max_distance = 0.35;
    double candidate_max_z_diff = 1.8;
    double bev_resolution = 0.5;
    double bev_search_xy_range = 4.0;
    double bev_search_xy_step = 0.5;
    double bev_search_yaw_range_deg = 12.0;
    double bev_search_yaw_step_deg = 3.0;
    double bev_min_score = 0.20;
    int bev_min_matched_points = 120;
    double icp_crop_padding = 1.0;
    int icp_min_source_points = 300;
    double icp_min_keep_ratio = 0.15;
};

struct RelocalizationResult
{
    bool success = false;
    int submap_id = -1;
    double ransac_fitness = 0.0;
    double ransac_rmse = 0.0;
    double icp_fitness = 0.0;
    double icp_rmse = 0.0;
    Eigen::Matrix4d map_to_odom = Eigen::Matrix4d::Identity();
    std::string message;
};

class Relocalizer
{
public:
    void SetOptions(const RelocalizerOptions &options);
    bool BuildSubmaps(const open3d::geometry::PointCloud &map, std::string *message = nullptr);

    RelocalizationResult Relocalize(
        const open3d::geometry::PointCloud &scan_in_odom,
        const Eigen::Matrix4d &predicted_map_to_odom,
        bool allow_large_jump) const;

    std::size_t SubmapCount() const;
    bool IsReady() const;

private:
    struct PolarDescriptor
    {
        int ring_count = 0;
        int sector_count = 0;
        double max_radius = 0.0;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        std::vector<double> values;
        std::vector<std::uint8_t> occupied;
        std::size_t occupied_bins = 0;
    };

    struct DescriptorMatch
    {
        double distance = 0.0;
        int sector_shift = 0;
    };

    struct BevGrid
    {
        double resolution = 0.0;
        Eigen::Vector2d min_bound = Eigen::Vector2d::Zero();
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> occupied;
    };

    struct MapSubmap
    {
        int id = -1;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        open3d::geometry::AxisAlignedBoundingBox aabb;
        std::shared_ptr<open3d::geometry::PointCloud> cloud;
        PolarDescriptor descriptor;
        BevGrid bev_grid;
    };

    struct DescriptorCandidate
    {
        int submap_index = -1;
        double distance = 0.0;
        double z_diff = 0.0;
        int sector_shift = 0;
    };

    struct BevCandidate
    {
        int submap_index = -1;
        double descriptor_distance = 0.0;
        double z_diff = 0.0;
        int sector_shift = 0;
        double yaw = 0.0;
        double score = 0.0;
        int matched_points = 0;
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    };

    bool OptionsValid(std::string *message) const;

    std::vector<DescriptorCandidate> BuildCandidateOrder(
        const PolarDescriptor &scan_descriptor,
        const Eigen::Vector3d &predicted_center_in_map,
        std::size_t *z_rejected_count) const;

    bool PreprocessCloud(
        const open3d::geometry::PointCloud &input,
        std::shared_ptr<open3d::geometry::PointCloud> &cloud) const;

    PolarDescriptor ComputeDescriptor(const open3d::geometry::PointCloud &cloud) const;
    DescriptorMatch DescriptorDistance(const PolarDescriptor &source, const PolarDescriptor &target) const;
    BevGrid BuildBevGrid(const open3d::geometry::PointCloud &cloud) const;
    BevCandidate SearchBevCoarseTransform(
        const open3d::geometry::PointCloud &source,
        const MapSubmap &submap,
        const DescriptorCandidate &candidate,
        const Eigen::Vector3d &source_center) const;
    std::shared_ptr<open3d::geometry::PointCloud> CropSourceForSubmap(
        const open3d::geometry::PointCloud &source,
        const Eigen::Matrix4d &source_to_map,
        const open3d::geometry::AxisAlignedBoundingBox &aabb,
        double padding,
        double *keep_ratio) const;

    double CandidateScore(double fitness, double rmse) const;

    RelocalizerOptions options_;
    std::vector<MapSubmap> submaps_;
};

} // namespace open3d_loc
