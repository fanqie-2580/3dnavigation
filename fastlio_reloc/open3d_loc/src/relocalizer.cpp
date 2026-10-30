#include "open3d_loc/relocalizer.h"

#include "open3d_registration/open3d_registration.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace open3d_loc
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct DescriptorDiagnostic
{
    int submap_index = -1;
    int submap_id = -1;
    double distance = 0.0;
    double z_diff = 0.0;
    int sector_shift = 0;
    double yaw_deg = 0.0;
};

struct BevDiagnostic
{
    int submap_index = -1;
    int submap_id = -1;
    double descriptor_distance = 0.0;
    double z_diff = 0.0;
    int sector_shift = 0;
    double yaw_deg = 0.0;
    double score = 0.0;
    int matched_points = 0;
    bool passed = false;
};

struct IcpDiagnostic
{
    int submap_index = -1;
    int submap_id = -1;
    double bev_score = 0.0;
    int bev_matched_points = 0;
    double icp_fitness = 0.0;
    double icp_rmse = 0.0;
    std::size_t crop_source_points = 0;
    double keep_ratio = 0.0;
};

double NormalizeAngle(double angle)
{
    while (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

double DegreesToRadians(double degrees)
{
    return degrees * kPi / 180.0;
}

double RadiansToDegrees(double radians)
{
    return radians * 180.0 / kPi;
}

double DescriptorShiftToYaw(int sector_shift, int sector_count)
{
    if (sector_count <= 0)
    {
        return 0.0;
    }
    const double sector_angle = 2.0 * kPi / static_cast<double>(sector_count);
    return NormalizeAngle(static_cast<double>(sector_shift) * sector_angle);
}

std::vector<double> BuildSearchOffsets(double range, double step)
{
    std::vector<double> offsets;
    if (range < 0.0 || step <= 0.0)
    {
        offsets.push_back(0.0);
        return offsets;
    }

    const int count = static_cast<int>(std::floor(range / step + 1e-9));
    offsets.reserve(static_cast<std::size_t>(count * 2 + 1));
    for (int i = -count; i <= count; ++i)
    {
        offsets.push_back(static_cast<double>(i) * step);
    }
    if (offsets.empty())
    {
        offsets.push_back(0.0);
    }
    return offsets;
}

std::string BuildFailureMessage(
    const std::string &reason,
    const std::string &failed_stage,
    std::size_t source_voxel_points,
    std::size_t z_passed,
    std::size_t z_rejected,
    std::size_t descriptor_candidates,
    std::size_t descriptor_selected,
    std::size_t bev_passed,
    const std::vector<DescriptorDiagnostic> &descriptor_diagnostics,
    const std::vector<BevDiagnostic> &bev_diagnostics,
    const std::vector<IcpDiagnostic> &icp_diagnostics,
    const RelocalizerOptions &options)
{
    std::size_t best_crop_source_points = 0;
    double best_keep_ratio = 0.0;
    for (const IcpDiagnostic &diagnostic : icp_diagnostics)
    {
        if (diagnostic.crop_source_points > best_crop_source_points)
        {
            best_crop_source_points = diagnostic.crop_source_points;
            best_keep_ratio = diagnostic.keep_ratio;
        }
    }

    std::ostringstream oss;
    oss << reason
        << " failed_stage=" << failed_stage
        << " source_voxel_points=" << source_voxel_points
        << " z_passed=" << z_passed
        << " z_rejected=" << z_rejected
        << " descriptor_candidates=" << descriptor_candidates
        << " descriptor_selected=" << descriptor_selected
        << " bev_passed=" << bev_passed
        << " crop_source_points=" << best_crop_source_points
        << " keep_ratio=" << best_keep_ratio;

    if (options.enable_diagnostics)
    {
        const int descriptor_top_count = std::min(
            std::max(0, options.diagnostics_top_n),
            static_cast<int>(descriptor_diagnostics.size()));
        oss << " top_descriptor=[";
        for (int i = 0; i < descriptor_top_count; ++i)
        {
            const DescriptorDiagnostic &diagnostic = descriptor_diagnostics[i];
            if (i > 0)
            {
                oss << "; ";
            }
            oss << "{submap_id=" << diagnostic.submap_id
                << " index=" << diagnostic.submap_index
                << " distance=" << diagnostic.distance
                << " z_diff=" << diagnostic.z_diff
                << " sector_shift=" << diagnostic.sector_shift
                << " yaw_deg=" << diagnostic.yaw_deg
                << "}";
        }
        oss << "]";

        const int bev_top_count = std::min(
            std::max(0, options.diagnostics_top_n),
            static_cast<int>(bev_diagnostics.size()));
        oss << " top_bev=[";
        for (int i = 0; i < bev_top_count; ++i)
        {
            const BevDiagnostic &diagnostic = bev_diagnostics[i];
            if (i > 0)
            {
                oss << "; ";
            }
            oss << "{submap_id=" << diagnostic.submap_id
                << " index=" << diagnostic.submap_index
                << " descriptor_distance=" << diagnostic.descriptor_distance
                << " z_diff=" << diagnostic.z_diff
                << " sector_shift=" << diagnostic.sector_shift
                << " yaw_deg=" << diagnostic.yaw_deg
                << " score=" << diagnostic.score
                << " matched_points=" << diagnostic.matched_points
                << " passed=" << (diagnostic.passed ? "true" : "false")
                << "}";
        }
        oss << "]";

        const int top_icp_count = std::min(
            std::max(0, options.diagnostics_top_n),
            static_cast<int>(icp_diagnostics.size()));
        oss << " top_icp=[";
        for (int i = 0; i < top_icp_count; ++i)
        {
            const IcpDiagnostic &diagnostic = icp_diagnostics[i];
            if (i > 0)
            {
                oss << "; ";
            }
            oss << "{submap_id=" << diagnostic.submap_id
                << " index=" << diagnostic.submap_index
                << " bev_score=" << diagnostic.bev_score
                << " bev_matched_points=" << diagnostic.bev_matched_points
                << " crop_source_points=" << diagnostic.crop_source_points
                << " keep_ratio=" << diagnostic.keep_ratio
                << " icp_fitness=" << diagnostic.icp_fitness
                << " icp_rmse=" << diagnostic.icp_rmse
                << "}";
        }
        oss << "]";
    }

    return oss.str();
}

} // namespace

void Relocalizer::SetOptions(const RelocalizerOptions &options)
{
    options_ = options;
}

bool Relocalizer::OptionsValid(std::string *message) const
{
    if (options_.submap_size.x() <= 0.0 || options_.submap_size.y() <= 0.0 || options_.submap_size.z() <= 0.0)
    {
        if (message)
        {
            *message = "submap_size must be positive";
        }
        return false;
    }
    if (options_.submap_stride.x() <= 0.0 || options_.submap_stride.y() <= 0.0 || options_.submap_stride.z() <= 0.0)
    {
        if (message)
        {
            *message = "submap_stride must be positive";
        }
        return false;
    }
    if (options_.voxel_size <= 0.0)
    {
        if (message)
        {
            *message = "voxel_size must be positive";
        }
        return false;
    }
    if (options_.min_submap_points <= 0 || options_.scan_min_points <= 0 || options_.top_k <= 0)
    {
        if (message)
        {
            *message = "point limits and top_k must be positive";
        }
        return false;
    }
    if (options_.diagnostics_top_n < 0)
    {
        if (message)
        {
            *message = "diagnostics_top_n must be non-negative";
        }
        return false;
    }
    if (options_.descriptor_ring_count <= 0 || options_.descriptor_sector_count <= 0 ||
        options_.descriptor_top_k <= 0)
    {
        if (message)
        {
            *message = "descriptor ring/sector counts and top_k must be positive";
        }
        return false;
    }
    if (options_.descriptor_max_radius <= 0.0 || options_.descriptor_max_distance < 0.0)
    {
        if (message)
        {
            *message = "descriptor radius must be positive and max distance must be non-negative";
        }
        return false;
    }
    if (options_.candidate_max_z_diff < 0.0)
    {
        if (message)
        {
            *message = "candidate_max_z_diff must be non-negative";
        }
        return false;
    }
    if (options_.bev_resolution <= 0.0 ||
        options_.bev_search_xy_range < 0.0 ||
        options_.bev_search_xy_step <= 0.0 ||
        options_.bev_search_yaw_range_deg < 0.0 ||
        options_.bev_search_yaw_step_deg <= 0.0 ||
        options_.bev_min_score < 0.0 ||
        options_.bev_min_score > 1.0 ||
        options_.bev_min_matched_points <= 0)
    {
        if (message)
        {
            *message = "BEV resolution, search ranges, thresholds, and min matched points are invalid";
        }
        return false;
    }
    if (options_.icp_crop_padding < 0.0 || options_.icp_min_source_points <= 0 ||
        options_.icp_min_keep_ratio <= 0.0 || options_.icp_min_keep_ratio > 1.0)
    {
        if (message)
        {
            *message = "ICP crop padding, min source points, and keep ratio are invalid";
        }
        return false;
    }
    return true;
}

bool Relocalizer::BuildSubmaps(const open3d::geometry::PointCloud &map, std::string *message)
{
    submaps_.clear();

    if (!OptionsValid(message))
    {
        return false;
    }
    if (map.IsEmpty())
    {
        if (message)
        {
            *message = "map is empty";
        }
        return false;
    }

    const Eigen::Vector3d min_bound = map.GetMinBound();
    const Eigen::Vector3d max_bound = map.GetMaxBound();
    int next_id = 0;

    for (double x = min_bound.x(); x <= max_bound.x(); x += options_.submap_stride.x())
    {
        for (double y = min_bound.y(); y <= max_bound.y(); y += options_.submap_stride.y())
        {
            for (double z = min_bound.z(); z <= max_bound.z(); z += options_.submap_stride.z())
            {
                const Eigen::Vector3d box_min(x, y, z);
                const Eigen::Vector3d box_max = box_min + options_.submap_size;
                open3d::geometry::AxisAlignedBoundingBox aabb(box_min, box_max);

                auto cropped = map.Crop(aabb);
                if (!cropped || cropped->points_.size() < static_cast<std::size_t>(options_.min_submap_points))
                {
                    continue;
                }

                auto down = cropped->VoxelDownSample(options_.voxel_size);
                if (!down || down->points_.size() < static_cast<std::size_t>(options_.min_submap_points))
                {
                    continue;
                }

                down->EstimateNormals(
                    open3d::geometry::KDTreeSearchParamHybrid(options_.voxel_size * 2.0, 30));
                down->OrientNormalsToAlignWithDirection();

                auto descriptor = ComputeDescriptor(*down);
                if (descriptor.occupied_bins == 0)
                {
                    continue;
                }

                auto bev_grid = BuildBevGrid(*down);
                if (bev_grid.occupied.empty())
                {
                    continue;
                }

                MapSubmap submap;
                submap.id = next_id++;
                submap.center = down->GetCenter();
                submap.aabb = aabb;
                submap.cloud = down;
                submap.descriptor = descriptor;
                submap.bev_grid = bev_grid;
                submaps_.push_back(submap);
            }
        }
    }

    if (submaps_.empty())
    {
        if (message)
        {
            *message = "no valid submaps generated";
        }
        return false;
    }

    if (message)
    {
        std::ostringstream oss;
        oss << "generated " << submaps_.size() << " submaps";
        *message = oss.str();
    }
    return true;
}

std::size_t Relocalizer::SubmapCount() const
{
    return submaps_.size();
}

bool Relocalizer::IsReady() const
{
    return !submaps_.empty();
}

bool Relocalizer::PreprocessCloud(
    const open3d::geometry::PointCloud &input,
    std::shared_ptr<open3d::geometry::PointCloud> &cloud) const
{
    if (input.IsEmpty())
    {
        return false;
    }

    cloud.reset(new open3d::geometry::PointCloud(input));
    *cloud = cloud->RemoveNonFinitePoints(true, true);
    cloud = cloud->VoxelDownSample(options_.voxel_size);
    if (!cloud || cloud->points_.size() < static_cast<std::size_t>(options_.scan_min_points))
    {
        return false;
    }

    cloud->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(options_.voxel_size * 2.0, 30));
    cloud->OrientNormalsToAlignWithDirection();

    return true;
}

Relocalizer::PolarDescriptor Relocalizer::ComputeDescriptor(
    const open3d::geometry::PointCloud &cloud) const
{
    PolarDescriptor descriptor;
    descriptor.ring_count = options_.descriptor_ring_count;
    descriptor.sector_count = options_.descriptor_sector_count;
    descriptor.max_radius = options_.descriptor_max_radius;

    if (cloud.IsEmpty() || descriptor.ring_count <= 0 || descriptor.sector_count <= 0 ||
        descriptor.max_radius <= 0.0)
    {
        return descriptor;
    }

    const int bin_count = descriptor.ring_count * descriptor.sector_count;
    descriptor.values.assign(static_cast<std::size_t>(bin_count), 0.0);
    descriptor.occupied.assign(static_cast<std::size_t>(bin_count), 0);
    descriptor.center = cloud.GetCenter();

    const Eigen::Vector3d min_bound = cloud.GetMinBound();
    const Eigen::Vector3d max_bound = cloud.GetMaxBound();
    const double height_scale = std::max(1.0, max_bound.z() - min_bound.z());

    for (const auto &point : cloud.points_)
    {
        const Eigen::Vector3d relative = point - descriptor.center;
        const double radius = std::hypot(relative.x(), relative.y());
        if (radius > descriptor.max_radius)
        {
            continue;
        }

        const int ring = std::min(
            descriptor.ring_count - 1,
            static_cast<int>(radius / descriptor.max_radius * descriptor.ring_count));

        double angle = std::atan2(relative.y(), relative.x());
        if (angle < 0.0)
        {
            angle += 2.0 * kPi;
        }
        const int sector = std::min(
            descriptor.sector_count - 1,
            static_cast<int>(angle / (2.0 * kPi) * descriptor.sector_count));
        const std::size_t index = static_cast<std::size_t>(ring * descriptor.sector_count + sector);
        const double normalized_height = std::min(
            1.0,
            std::max(0.0, (point.z() - min_bound.z()) / height_scale));

        if (descriptor.occupied[index] == 0)
        {
            descriptor.occupied[index] = 1;
            descriptor.values[index] = normalized_height;
            ++descriptor.occupied_bins;
        }
        else
        {
            descriptor.values[index] = std::max(descriptor.values[index], normalized_height);
        }
    }

    return descriptor;
}

Relocalizer::DescriptorMatch Relocalizer::DescriptorDistance(
    const PolarDescriptor &source,
    const PolarDescriptor &target) const
{
    DescriptorMatch best_match;
    best_match.distance = std::numeric_limits<double>::infinity();
    best_match.sector_shift = 0;

    if (source.ring_count <= 0 || source.sector_count <= 0 ||
        source.ring_count != target.ring_count ||
        source.sector_count != target.sector_count ||
        source.occupied_bins == 0 || target.occupied_bins == 0 ||
        source.values.size() != target.values.size() ||
        source.occupied.size() != target.occupied.size())
    {
        return best_match;
    }

    for (int shift = 0; shift < source.sector_count; ++shift)
    {
        std::size_t source_bins = 0;
        std::size_t matched_bins = 0;
        double height_error = 0.0;

        for (int ring = 0; ring < source.ring_count; ++ring)
        {
            for (int sector = 0; sector < source.sector_count; ++sector)
            {
                const std::size_t source_index =
                    static_cast<std::size_t>(ring * source.sector_count + sector);
                if (source.occupied[source_index] == 0)
                {
                    continue;
                }

                ++source_bins;
                const int target_sector = (sector + shift) % source.sector_count;
                const std::size_t target_index =
                    static_cast<std::size_t>(ring * source.sector_count + target_sector);
                if (target.occupied[target_index] == 0)
                {
                    continue;
                }

                ++matched_bins;
                height_error += std::abs(source.values[source_index] - target.values[target_index]);
            }
        }

        if (source_bins == 0)
        {
            continue;
        }

        const double occupancy_distance =
            1.0 - static_cast<double>(matched_bins) / static_cast<double>(source_bins);
        const double height_distance =
            matched_bins > 0 ? height_error / static_cast<double>(matched_bins) : 1.0;
        const double distance = 0.85 * occupancy_distance + 0.15 * height_distance;
        if (distance < best_match.distance)
        {
            best_match.distance = distance;
            best_match.sector_shift = shift;
        }
    }

    return best_match;
}

Relocalizer::BevGrid Relocalizer::BuildBevGrid(
    const open3d::geometry::PointCloud &cloud) const
{
    BevGrid grid;
    grid.resolution = options_.bev_resolution;
    if (cloud.IsEmpty() || grid.resolution <= 0.0)
    {
        return grid;
    }

    const Eigen::Vector3d min_bound_3d = cloud.GetMinBound();
    const Eigen::Vector3d max_bound_3d = cloud.GetMaxBound();
    if (!std::isfinite(min_bound_3d.x()) || !std::isfinite(min_bound_3d.y()) ||
        !std::isfinite(max_bound_3d.x()) || !std::isfinite(max_bound_3d.y()) ||
        max_bound_3d.x() < min_bound_3d.x() || max_bound_3d.y() < min_bound_3d.y())
    {
        return grid;
    }

    grid.min_bound = Eigen::Vector2d(min_bound_3d.x(), min_bound_3d.y());
    grid.width = std::max(1, static_cast<int>(std::floor((max_bound_3d.x() - min_bound_3d.x()) / grid.resolution)) + 1);
    grid.height = std::max(1, static_cast<int>(std::floor((max_bound_3d.y() - min_bound_3d.y()) / grid.resolution)) + 1);
    grid.occupied.assign(static_cast<std::size_t>(grid.width * grid.height), 0);

    std::size_t occupied_count = 0;
    for (const auto &point : cloud.points_)
    {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()))
        {
            continue;
        }

        const int x = static_cast<int>(std::floor((point.x() - grid.min_bound.x()) / grid.resolution));
        const int y = static_cast<int>(std::floor((point.y() - grid.min_bound.y()) / grid.resolution));
        if (x < 0 || x >= grid.width || y < 0 || y >= grid.height)
        {
            continue;
        }

        const std::size_t index = static_cast<std::size_t>(y * grid.width + x);
        if (grid.occupied[index] == 0)
        {
            grid.occupied[index] = 1;
            ++occupied_count;
        }
    }

    if (occupied_count == 0)
    {
        grid.width = 0;
        grid.height = 0;
        grid.occupied.clear();
    }
    return grid;
}

Relocalizer::BevCandidate Relocalizer::SearchBevCoarseTransform(
    const open3d::geometry::PointCloud &source,
    const MapSubmap &submap,
    const DescriptorCandidate &candidate,
    const Eigen::Vector3d &source_center) const
{
    BevCandidate best;
    best.submap_index = candidate.submap_index;
    best.descriptor_distance = candidate.distance;
    best.z_diff = candidate.z_diff;
    best.sector_shift = candidate.sector_shift;
    best.score = -std::numeric_limits<double>::infinity();

    const BevGrid &grid = submap.bev_grid;
    if (source.points_.empty() || grid.occupied.empty() ||
        grid.resolution <= 0.0 || grid.width <= 0 || grid.height <= 0)
    {
        best.score = 0.0;
        return best;
    }

    const double yaw_center = DescriptorShiftToYaw(candidate.sector_shift, submap.descriptor.sector_count);
    const std::vector<double> yaw_offsets = BuildSearchOffsets(
        DegreesToRadians(options_.bev_search_yaw_range_deg),
        DegreesToRadians(options_.bev_search_yaw_step_deg));
    const std::vector<double> xy_offsets = BuildSearchOffsets(
        options_.bev_search_xy_range,
        options_.bev_search_xy_step);

    double best_offset_cost = std::numeric_limits<double>::infinity();
    for (const double yaw_offset : yaw_offsets)
    {
        const double yaw = NormalizeAngle(yaw_center + yaw_offset);
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);

        const Eigen::Vector3d rotated_source_center(
            cos_yaw * source_center.x() - sin_yaw * source_center.y(),
            sin_yaw * source_center.x() + cos_yaw * source_center.y(),
            source_center.z());
        Eigen::Vector3d base_translation = submap.center - rotated_source_center;

        for (const double dx : xy_offsets)
        {
            for (const double dy : xy_offsets)
            {
                const double tx = base_translation.x() + dx;
                const double ty = base_translation.y() + dy;

                int matched_points = 0;
                for (const auto &point : source.points_)
                {
                    if (!std::isfinite(point.x()) || !std::isfinite(point.y()))
                    {
                        continue;
                    }

                    const double x = cos_yaw * point.x() - sin_yaw * point.y() + tx;
                    const double y = sin_yaw * point.x() + cos_yaw * point.y() + ty;
                    const int cell_x = static_cast<int>(std::floor((x - grid.min_bound.x()) / grid.resolution));
                    const int cell_y = static_cast<int>(std::floor((y - grid.min_bound.y()) / grid.resolution));
                    if (cell_x < 0 || cell_x >= grid.width || cell_y < 0 || cell_y >= grid.height)
                    {
                        continue;
                    }

                    const std::size_t index = static_cast<std::size_t>(cell_y * grid.width + cell_x);
                    if (grid.occupied[index] != 0)
                    {
                        ++matched_points;
                    }
                }

                const double score =
                    static_cast<double>(matched_points) / static_cast<double>(source.points_.size());
                const double offset_cost = std::abs(yaw_offset) + std::hypot(dx, dy);
                if (score > best.score ||
                    (score == best.score && matched_points > best.matched_points) ||
                    (score == best.score && matched_points == best.matched_points && offset_cost < best_offset_cost))
                {
                    best.score = score;
                    best.matched_points = matched_points;
                    best.yaw = yaw;
                    best_offset_cost = offset_cost;
                    best.transform = Eigen::Matrix4d::Identity();
                    best.transform(0, 0) = cos_yaw;
                    best.transform(0, 1) = -sin_yaw;
                    best.transform(1, 0) = sin_yaw;
                    best.transform(1, 1) = cos_yaw;
                    best.transform(0, 3) = tx;
                    best.transform(1, 3) = ty;
                    best.transform(2, 3) = base_translation.z();
                }
            }
        }
    }

    if (!std::isfinite(best.score))
    {
        best.score = 0.0;
    }
    return best;
}

std::vector<Relocalizer::DescriptorCandidate> Relocalizer::BuildCandidateOrder(
    const PolarDescriptor &scan_descriptor,
    const Eigen::Vector3d &predicted_center_in_map,
    std::size_t *z_rejected_count) const
{
    if (z_rejected_count)
    {
        *z_rejected_count = 0;
    }

    std::vector<DescriptorCandidate> candidates;
    for (std::size_t i = 0; i < submaps_.size(); ++i)
    {
        const MapSubmap &submap = submaps_[i];
        const double predicted_distance = (submap.center - predicted_center_in_map).norm();
        if (!options_.enable_global_search && predicted_distance > options_.nearby_radius)
        {
            continue;
        }

        const double z_diff = std::abs(submap.center.z() - predicted_center_in_map.z());
        if (z_diff > options_.candidate_max_z_diff)
        {
            if (z_rejected_count)
            {
                ++(*z_rejected_count);
            }
            continue;
        }

        const DescriptorMatch descriptor_match = DescriptorDistance(scan_descriptor, submap.descriptor);
        if (!std::isfinite(descriptor_match.distance))
        {
            continue;
        }

        DescriptorCandidate candidate;
        candidate.submap_index = static_cast<int>(i);
        candidate.distance = descriptor_match.distance;
        candidate.z_diff = z_diff;
        candidate.sector_shift = descriptor_match.sector_shift;
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const DescriptorCandidate &lhs, const DescriptorCandidate &rhs) {
                  if (lhs.distance == rhs.distance)
                  {
                      return lhs.z_diff < rhs.z_diff;
                  }
                  return lhs.distance < rhs.distance;
              });

    return candidates;
}

std::shared_ptr<open3d::geometry::PointCloud> Relocalizer::CropSourceForSubmap(
    const open3d::geometry::PointCloud &source,
    const Eigen::Matrix4d &source_to_map,
    const open3d::geometry::AxisAlignedBoundingBox &aabb,
    double padding,
    double *keep_ratio) const
{
    if (keep_ratio)
    {
        *keep_ratio = 0.0;
    }

    std::shared_ptr<open3d::geometry::PointCloud> cropped(new open3d::geometry::PointCloud);
    if (source.points_.empty())
    {
        return cropped;
    }

    const Eigen::Vector3d min_bound = aabb.GetMinBound() - Eigen::Vector3d::Constant(padding);
    const Eigen::Vector3d max_bound = aabb.GetMaxBound() + Eigen::Vector3d::Constant(padding);
    const bool copy_normals = source.normals_.size() == source.points_.size();
    const bool copy_colors = source.colors_.size() == source.points_.size();

    for (std::size_t i = 0; i < source.points_.size(); ++i)
    {
        const Eigen::Vector3d transformed =
            (source_to_map * source.points_[i].homogeneous()).hnormalized();
        if (!std::isfinite(transformed.x()) ||
            !std::isfinite(transformed.y()) ||
            !std::isfinite(transformed.z()))
        {
            continue;
        }

        if (transformed.x() < min_bound.x() || transformed.x() > max_bound.x() ||
            transformed.y() < min_bound.y() || transformed.y() > max_bound.y() ||
            transformed.z() < min_bound.z() || transformed.z() > max_bound.z())
        {
            continue;
        }

        cropped->points_.push_back(source.points_[i]);
        if (copy_normals)
        {
            cropped->normals_.push_back(source.normals_[i]);
        }
        if (copy_colors)
        {
            cropped->colors_.push_back(source.colors_[i]);
        }
    }

    if (keep_ratio)
    {
        *keep_ratio = static_cast<double>(cropped->points_.size()) /
                      static_cast<double>(source.points_.size());
    }
    return cropped;
}

double Relocalizer::CandidateScore(double fitness, double rmse) const
{
    return fitness - 0.2 * rmse;
}

RelocalizationResult Relocalizer::Relocalize(
    const open3d::geometry::PointCloud &scan_in_odom,
    const Eigen::Matrix4d &predicted_map_to_odom,
    bool allow_large_jump) const
{
    RelocalizationResult result;

    std::string options_message;
    if (!OptionsValid(&options_message))
    {
        result.message = BuildFailureMessage(
            "invalid options: " + options_message,
            "options",
            0,
            0,
            0,
            0,
            0,
            0,
            std::vector<DescriptorDiagnostic>(),
            std::vector<BevDiagnostic>(),
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    if (submaps_.empty())
    {
        result.message = BuildFailureMessage(
            "submap library is empty",
            "not_ready",
            0,
            0,
            0,
            0,
            0,
            0,
            std::vector<DescriptorDiagnostic>(),
            std::vector<BevDiagnostic>(),
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    std::shared_ptr<open3d::geometry::PointCloud> scan;
    if (!PreprocessCloud(scan_in_odom, scan))
    {
        const std::size_t source_voxel_points = scan ? scan->points_.size() : 0;
        result.message = BuildFailureMessage(
            "scan preprocessing failed",
            "preprocess",
            source_voxel_points,
            0,
            0,
            0,
            0,
            0,
            std::vector<DescriptorDiagnostic>(),
            std::vector<BevDiagnostic>(),
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    const std::size_t source_voxel_points = scan->points_.size();
    const Eigen::Vector3d scan_center_odom = scan->GetCenter();
    const Eigen::Vector3d predicted_center_map =
        (predicted_map_to_odom * scan_center_odom.homogeneous()).hnormalized();
    const PolarDescriptor scan_descriptor = ComputeDescriptor(*scan);
    if (scan_descriptor.occupied_bins == 0)
    {
        result.message = BuildFailureMessage(
            "scan descriptor is empty",
            "descriptor_preprocess",
            source_voxel_points,
            0,
            0,
            0,
            0,
            0,
            std::vector<DescriptorDiagnostic>(),
            std::vector<BevDiagnostic>(),
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    std::size_t z_rejected_count = 0;
    const std::vector<DescriptorCandidate> descriptor_candidates =
        BuildCandidateOrder(scan_descriptor, predicted_center_map, &z_rejected_count);

    std::vector<DescriptorDiagnostic> descriptor_diagnostics;
    descriptor_diagnostics.reserve(descriptor_candidates.size());
    for (const DescriptorCandidate &candidate : descriptor_candidates)
    {
        const MapSubmap &submap = submaps_[candidate.submap_index];
        DescriptorDiagnostic diagnostic;
        diagnostic.submap_index = candidate.submap_index;
        diagnostic.submap_id = submap.id;
        diagnostic.distance = candidate.distance;
        diagnostic.z_diff = candidate.z_diff;
        diagnostic.sector_shift = candidate.sector_shift;
        diagnostic.yaw_deg = RadiansToDegrees(
            DescriptorShiftToYaw(candidate.sector_shift, submap.descriptor.sector_count));
        descriptor_diagnostics.push_back(diagnostic);
    }

    const std::size_t z_passed_count = descriptor_candidates.size();
    if (descriptor_candidates.empty())
    {
        result.message = BuildFailureMessage(
            z_rejected_count > 0 ? "no submap candidates after z filtering" : "no descriptor candidates selected",
            z_rejected_count > 0 ? "candidate_z_filter" : "descriptor_candidates",
            source_voxel_points,
            z_passed_count,
            z_rejected_count,
            0,
            0,
            0,
            descriptor_diagnostics,
            std::vector<BevDiagnostic>(),
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    std::vector<DescriptorCandidate> selected_descriptor_candidates;
    for (const DescriptorCandidate &candidate : descriptor_candidates)
    {
        if (candidate.distance > options_.descriptor_max_distance)
        {
            continue;
        }
        if (selected_descriptor_candidates.size() < static_cast<std::size_t>(options_.descriptor_top_k))
        {
            selected_descriptor_candidates.push_back(candidate);
        }
    }

    if (selected_descriptor_candidates.empty())
    {
        result.message = BuildFailureMessage(
            "no descriptor candidate passed distance threshold",
            "descriptor_threshold",
            source_voxel_points,
            z_passed_count,
            z_rejected_count,
            descriptor_candidates.size(),
            selected_descriptor_candidates.size(),
            0,
            descriptor_diagnostics,
            std::vector<BevDiagnostic>(),
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    std::vector<BevCandidate> bev_candidates;
    std::vector<BevDiagnostic> bev_diagnostics;
    for (const DescriptorCandidate &descriptor_candidate : selected_descriptor_candidates)
    {
        const MapSubmap &submap = submaps_[descriptor_candidate.submap_index];
        BevCandidate bev_candidate = SearchBevCoarseTransform(
            *scan,
            submap,
            descriptor_candidate,
            scan_center_odom);

        BevDiagnostic diagnostic;
        diagnostic.submap_index = descriptor_candidate.submap_index;
        diagnostic.submap_id = submap.id;
        diagnostic.descriptor_distance = descriptor_candidate.distance;
        diagnostic.z_diff = descriptor_candidate.z_diff;
        diagnostic.sector_shift = descriptor_candidate.sector_shift;
        diagnostic.yaw_deg = RadiansToDegrees(bev_candidate.yaw);
        diagnostic.score = bev_candidate.score;
        diagnostic.matched_points = bev_candidate.matched_points;
        diagnostic.passed =
            bev_candidate.score >= options_.bev_min_score &&
            bev_candidate.matched_points >= options_.bev_min_matched_points;
        bev_diagnostics.push_back(diagnostic);

        if (!diagnostic.passed)
        {
            continue;
        }
        bev_candidates.push_back(bev_candidate);
    }

    std::sort(bev_diagnostics.begin(), bev_diagnostics.end(),
              [](const BevDiagnostic &lhs, const BevDiagnostic &rhs) {
                  if (lhs.score == rhs.score)
                  {
                      return lhs.matched_points > rhs.matched_points;
                  }
                  return lhs.score > rhs.score;
              });

    if (bev_candidates.empty())
    {
        result.message = BuildFailureMessage(
            "no BEV coarse candidate passed thresholds",
            "bev_threshold",
            source_voxel_points,
            z_passed_count,
            z_rejected_count,
            descriptor_candidates.size(),
            selected_descriptor_candidates.size(),
            0,
            descriptor_diagnostics,
            bev_diagnostics,
            std::vector<IcpDiagnostic>(),
            options_);
        return result;
    }

    std::sort(bev_candidates.begin(), bev_candidates.end(),
              [](const BevCandidate &lhs, const BevCandidate &rhs) {
                  if (lhs.score == rhs.score)
                  {
                      return lhs.matched_points > rhs.matched_points;
                  }
                  return lhs.score > rhs.score;
              });

    const int top_k = std::min(options_.top_k, static_cast<int>(bev_candidates.size()));
    double best_score = -std::numeric_limits<double>::infinity();
    double best_bev_score = 0.0;
    int best_bev_matched_points = 0;
    std::size_t best_crop_source_points = 0;
    double best_keep_ratio = 0.0;
    std::vector<IcpDiagnostic> icp_diagnostics;

    for (int i = 0; i < top_k; ++i)
    {
        const BevCandidate &candidate = bev_candidates[i];
        const MapSubmap &submap = submaps_[candidate.submap_index];

        double keep_ratio = 0.0;
        auto cropped_scan = CropSourceForSubmap(
            *scan,
            candidate.transform,
            submap.aabb,
            options_.icp_crop_padding,
            &keep_ratio);

        IcpDiagnostic icp_diagnostic;
        icp_diagnostic.submap_index = candidate.submap_index;
        icp_diagnostic.submap_id = submap.id;
        icp_diagnostic.bev_score = candidate.score;
        icp_diagnostic.bev_matched_points = candidate.matched_points;
        icp_diagnostic.crop_source_points = cropped_scan ? cropped_scan->points_.size() : 0;
        icp_diagnostic.keep_ratio = keep_ratio;

        if (!cropped_scan ||
            cropped_scan->points_.size() < static_cast<std::size_t>(options_.icp_min_source_points) ||
            keep_ratio < options_.icp_min_keep_ratio)
        {
            icp_diagnostic.icp_fitness = 0.0;
            icp_diagnostic.icp_rmse = std::numeric_limits<double>::infinity();
            icp_diagnostics.push_back(icp_diagnostic);
            continue;
        }

        auto icp = pcd_tools::RegistrationIcp(
            cropped_scan,
            submap.cloud,
            static_cast<float>(options_.voxel_size * 1.5),
            candidate.transform,
            1,
            30);

        const Eigen::Matrix4d refined = icp.transformation_ * candidate.transform;
        auto evaluation = open3d::pipelines::registration::EvaluateRegistration(
            *cropped_scan,
            *submap.cloud,
            options_.voxel_size * 1.5,
            refined);

        icp_diagnostic.icp_fitness = evaluation.fitness_;
        icp_diagnostic.icp_rmse = evaluation.inlier_rmse_;
        icp_diagnostics.push_back(icp_diagnostic);

        if (evaluation.fitness_ < options_.icp_min_fitness ||
            evaluation.inlier_rmse_ > options_.icp_max_rmse)
        {
            continue;
        }

        if (!allow_large_jump)
        {
            const Eigen::Vector3d refined_center_map =
                (refined * scan_center_odom.homogeneous()).hnormalized();
            if ((refined_center_map - predicted_center_map).norm() > options_.max_pose_jump)
            {
                continue;
            }
        }

        const double score = CandidateScore(evaluation.fitness_, evaluation.inlier_rmse_);
        if (score > best_score)
        {
            best_score = score;
            result.success = true;
            result.submap_id = submap.id;
            result.ransac_fitness = 0.0;
            result.ransac_rmse = 0.0;
            result.icp_fitness = evaluation.fitness_;
            result.icp_rmse = evaluation.inlier_rmse_;
            result.map_to_odom = refined;
            best_bev_score = candidate.score;
            best_bev_matched_points = candidate.matched_points;
            best_crop_source_points = icp_diagnostic.crop_source_points;
            best_keep_ratio = icp_diagnostic.keep_ratio;
        }
    }

    if (!result.success)
    {
        std::sort(icp_diagnostics.begin(), icp_diagnostics.end(),
                  [this](const IcpDiagnostic &lhs, const IcpDiagnostic &rhs) {
                      return CandidateScore(lhs.icp_fitness, lhs.icp_rmse) >
                             CandidateScore(rhs.icp_fitness, rhs.icp_rmse);
                  });
        result.message = BuildFailureMessage(
            "no ICP candidate passed thresholds",
            "icp_validation",
            source_voxel_points,
            z_passed_count,
            z_rejected_count,
            descriptor_candidates.size(),
            selected_descriptor_candidates.size(),
            bev_candidates.size(),
            descriptor_diagnostics,
            bev_diagnostics,
            icp_diagnostics,
            options_);
        return result;
    }

    std::ostringstream oss;
    oss << "submap " << result.submap_id
        << " bev_score=" << best_bev_score
        << " bev_matched_points=" << best_bev_matched_points
        << " crop_source_points=" << best_crop_source_points
        << " keep_ratio=" << best_keep_ratio
        << " icp_fitness=" << result.icp_fitness
        << " icp_rmse=" << result.icp_rmse;
    result.message = oss.str();
    return result;
}

} // namespace open3d_loc
