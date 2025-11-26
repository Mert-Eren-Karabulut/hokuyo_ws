#include "hokuyo_go/modular_scanner/voxel_grid.h"

namespace hokuyo_go
{

float SUBVOXEL_THRESHOLD = 0.35f;

// ========================================
// VoxelPoint Implementation
// ========================================

VoxelPoint::VoxelPoint() 
    : x(0), y(0), z(0), sensor_distance(0), count(0), subdivision_level(0) {}

VoxelPoint::VoxelPoint(float x_, float y_, float z_, float dist, int level)
    : x(x_), y(y_), z(z_), sensor_distance(dist), count(1), subdivision_level(level) {}

void VoxelPoint::updatePosition(float new_x, float new_y, float new_z, float new_distance)
{
    float new_count = count + 1;
    x = (x * count + new_x) / new_count;
    y = (y * count + new_y) / new_count;
    z = (z * count + new_z) / new_count;
    count++;

    if (new_distance < sensor_distance)
    {
        sensor_distance = new_distance;
    }
}

void VoxelPoint::getColor(uint8_t &r, uint8_t &g, uint8_t &b) const
{
    switch (subdivision_level)
    {
    case 0:
        r = 50; g = 50; b = 255; // Blue
        break;
    case 1:
        r = 50; g = 255; b = 50; // Green
        break;
    case 2:
        r = 255; g = 255; b = 50; // Yellow
        break;
    case 3:
    default:
        r = 255; g = 50; b = 50; // Red
        break;
    }
}

// ========================================
// SubVoxel Implementation
// ========================================

SubVoxel::SubVoxel() 
    : current_level(0), is_subdivided(false),
      min_x(0), min_y(0), min_z(0),
      max_x(0), max_y(0), max_z(0) {}

SubVoxel::SubVoxel(float minx, float miny, float minz, float maxx, float maxy, float maxz, int level)
    : current_level(level), is_subdivided(false),
      min_x(minx), min_y(miny), min_z(minz),
      max_x(maxx), max_y(maxy), max_z(maxz) {}

bool SubVoxel::containsPoint(float x, float y, float z) const
{
    return x >= min_x && x < max_x &&
           y >= min_y && y < max_y &&
           z >= min_z && z < max_z;
}

int SubVoxel::getChildIndex(float x, float y, float z) const
{
    float mid_x = (min_x + max_x) / 2.0f;
    float mid_y = (min_y + max_y) / 2.0f;
    float mid_z = (min_z + max_z) / 2.0f;

    int idx = 0;
    if (x >= mid_x) idx |= 1;
    if (y >= mid_y) idx |= 2;
    if (z >= mid_z) idx |= 4;
    return idx;
}

void SubVoxel::subdivide()
{
    if (is_subdivided || current_level >= MAX_SUBVOXEL_LEVEL)
        return;

    float mid_x = (min_x + max_x) / 2.0f;
    float mid_y = (min_y + max_y) / 2.0f;
    float mid_z = (min_z + max_z) / 2.0f;

    children.resize(8);
    int next_level = current_level + 1;

    children[0] = SubVoxel(min_x, min_y, min_z, mid_x, mid_y, mid_z, next_level);
    children[1] = SubVoxel(mid_x, min_y, min_z, max_x, mid_y, mid_z, next_level);
    children[2] = SubVoxel(min_x, mid_y, min_z, mid_x, max_y, mid_z, next_level);
    children[3] = SubVoxel(mid_x, mid_y, min_z, max_x, max_y, mid_z, next_level);
    children[4] = SubVoxel(min_x, min_y, mid_z, mid_x, mid_y, max_z, next_level);
    children[5] = SubVoxel(mid_x, min_y, mid_z, max_x, mid_y, max_z, next_level);
    children[6] = SubVoxel(min_x, mid_y, mid_z, mid_x, max_y, max_z, next_level);
    children[7] = SubVoxel(mid_x, mid_y, mid_z, max_x, max_y, max_z, next_level);

    is_subdivided = true;
}

void SubVoxel::unsubdivide()
{
    if (!is_subdivided)
        return;

    int total_count = 0;
    float total_x = 0, total_y = 0, total_z = 0;
    float min_distance = std::numeric_limits<float>::max();

    if (point_data.count > 0)
    {
        total_x += point_data.x * point_data.count;
        total_y += point_data.y * point_data.count;
        total_z += point_data.z * point_data.count;
        total_count += point_data.count;
        min_distance = point_data.sensor_distance;
    }

    for (auto &child : children)
    {
        if (child.point_data.count > 0)
        {
            total_x += child.point_data.x * child.point_data.count;
            total_y += child.point_data.y * child.point_data.count;
            total_z += child.point_data.z * child.point_data.count;
            total_count += child.point_data.count;
            min_distance = std::min(min_distance, child.point_data.sensor_distance);
        }
    }

    if (total_count > 0)
    {
        point_data.x = total_x / total_count;
        point_data.y = total_y / total_count;
        point_data.z = total_z / total_count;
        point_data.count = total_count;
        point_data.sensor_distance = min_distance;
    }

    children.clear();
    is_subdivided = false;
}

// ========================================
// ParentVoxel Implementation
// ========================================

ParentVoxel::ParentVoxel() 
    : is_subdivided(false),
      min_x(0), min_y(0), min_z(0),
      max_x(0), max_y(0), max_z(0),
      total_points(0) {}

ParentVoxel::ParentVoxel(float minx, float miny, float minz, float voxel_size)
    : is_subdivided(false),
      min_x(minx), min_y(miny), min_z(minz),
      max_x(minx + voxel_size), max_y(miny + voxel_size), max_z(minz + voxel_size),
      total_points(0)
{
    subvoxels.resize(8);
    float mid_x = (min_x + max_x) / 2.0f;
    float mid_y = (min_y + max_y) / 2.0f;
    float mid_z = (min_z + max_z) / 2.0f;

    subvoxels[0] = SubVoxel(min_x, min_y, min_z, mid_x, mid_y, mid_z, 1);
    subvoxels[1] = SubVoxel(mid_x, min_y, min_z, max_x, mid_y, mid_z, 1);
    subvoxels[2] = SubVoxel(min_x, mid_y, min_z, mid_x, max_y, mid_z, 1);
    subvoxels[3] = SubVoxel(mid_x, mid_y, min_z, max_x, max_y, mid_z, 1);
    subvoxels[4] = SubVoxel(min_x, min_y, mid_z, mid_x, mid_y, max_z, 1);
    subvoxels[5] = SubVoxel(mid_x, min_y, mid_z, max_x, mid_y, max_z, 1);
    subvoxels[6] = SubVoxel(min_x, mid_y, mid_z, mid_x, max_y, max_z, 1);
    subvoxels[7] = SubVoxel(mid_x, mid_y, mid_z, max_x, max_y, max_z, 1);
}

void ParentVoxel::insertPoint(float x, float y, float z, float distance)
{
    total_points++;
    averaged_point.updatePosition(x, y, z, distance);

    int subvoxel_idx = getSubvoxelIndex(x, y, z);
    if (subvoxel_idx < 0 || subvoxel_idx >= 8)
        return;

    SubVoxel &subvox = subvoxels[subvoxel_idx];
    insertIntoSubvoxel(subvox, x, y, z, distance);

    if (total_points % 10 == 0)
    {
        evaluateSubdivision();
    }
}

int ParentVoxel::getSubvoxelIndex(float x, float y, float z) const
{
    float mid_x = (min_x + max_x) / 2.0f;
    float mid_y = (min_y + max_y) / 2.0f;
    float mid_z = (min_z + max_z) / 2.0f;

    int idx = 0;
    if (x >= mid_x) idx |= 1;
    if (y >= mid_y) idx |= 2;
    if (z >= mid_z) idx |= 4;
    return idx;
}

void ParentVoxel::insertIntoSubvoxel(SubVoxel &subvox, float x, float y, float z, float distance)
{
    if (subvox.is_subdivided)
    {
        int child_idx = subvox.getChildIndex(x, y, z);
        if (child_idx >= 0 && child_idx < 8)
        {
            insertIntoSubvoxel(subvox.children[child_idx], x, y, z, distance);
        }
    }
    else
    {
        subvox.point_data.updatePosition(x, y, z, distance);
    }
}

void ParentVoxel::evaluateSubdivision()
{
    int subvoxels_above_threshold = 0;
    for (const auto &subvox : subvoxels)
    {
        float ratio = (total_points > 0) ? (float)subvox.point_data.count / (float)total_points : 0.0f;
        if (ratio >= SUBVOXEL_THRESHOLD)
        {
            subvoxels_above_threshold++;
        }
    }

    is_subdivided = (subvoxels_above_threshold > 0);

    for (auto &subvox : subvoxels)
    {
        evaluateSubvoxelSubdivision(subvox);
    }
}

void ParentVoxel::evaluateSubvoxelSubdivision(SubVoxel &subvox)
{
    if (subvox.current_level >= MAX_SUBVOXEL_LEVEL)
        return;

    if (subvox.point_data.count == 0)
        return;

    if (!subvox.is_subdivided)
    {
        float ratio = (total_points > 0) ? (float)subvox.point_data.count / (float)total_points : 0.0f;
        int min_points_to_subdivide = 5 * (subvox.current_level + 1);

        if (ratio >= SUBVOXEL_THRESHOLD && subvox.point_data.count >= min_points_to_subdivide)
        {
            subvox.subdivide();
        }
    }
    else
    {
        int child_points = 0;
        int children_with_points = 0;
        int max_child_points = 0;

        for (const auto &child : subvox.children)
        {
            int cp = child.point_data.count;
            child_points += cp;
            if (cp > 0) children_with_points++;
            if (cp > max_child_points) max_child_points = cp;
        }

        if (child_points > 0)
        {
            float max_child_ratio = (float)max_child_points / (float)child_points;

            if (max_child_ratio >= SUBVOXEL_THRESHOLD)
            {
                for (auto &child : subvox.children)
                {
                    if (child.point_data.count >= 3)
                    {
                        evaluateSubvoxelSubdivisionRecursive(child, child_points);
                    }
                }
            }
            else
            {
                subvox.unsubdivide();
            }
        }
    }
}

void ParentVoxel::evaluateSubvoxelSubdivisionRecursive(SubVoxel &subvox, int parent_total)
{
    if (subvox.current_level >= MAX_SUBVOXEL_LEVEL)
        return;

    if (subvox.point_data.count == 0)
        return;

    if (!subvox.is_subdivided)
    {
        float ratio = (parent_total > 0) ? (float)subvox.point_data.count / (float)parent_total : 0.0f;
        int min_points = 3 * (subvox.current_level + 1);

        if (ratio >= SUBVOXEL_THRESHOLD && subvox.point_data.count >= min_points)
        {
            subvox.subdivide();
        }
    }
    else
    {
        int child_points = 0;
        int max_child_points = 0;

        for (const auto &child : subvox.children)
        {
            int cp = child.point_data.count;
            child_points += cp;
            if (cp > max_child_points) max_child_points = cp;
        }

        if (child_points > 0)
        {
            float max_child_ratio = (float)max_child_points / (float)child_points;

            if (max_child_ratio >= SUBVOXEL_THRESHOLD)
            {
                for (auto &child : subvox.children)
                {
                    if (child.point_data.count >= 2)
                    {
                        evaluateSubvoxelSubdivisionRecursive(child, child_points);
                    }
                }
            }
            else
            {
                subvox.unsubdivide();
            }
        }
    }
}

void ParentVoxel::collectPoints(std::vector<VoxelPoint> &points) const
{
    if (!is_subdivided)
    {
        if (averaged_point.count > 0)
        {
            VoxelPoint pt = averaged_point;
            pt.subdivision_level = 0;
            points.push_back(pt);
        }
    }
    else
    {
        for (const auto &subvox : subvoxels)
        {
            collectSubvoxelPoints(subvox, points);
        }
    }
}

void ParentVoxel::collectSubvoxelPoints(const SubVoxel &subvox, std::vector<VoxelPoint> &points) const
{
    if (subvox.is_subdivided)
    {
        for (const auto &child : subvox.children)
        {
            collectSubvoxelPoints(child, points);
        }
    }
    else
    {
        if (subvox.point_data.count > 0)
        {
            VoxelPoint pt = subvox.point_data;
            pt.subdivision_level = subvox.current_level;
            points.push_back(pt);
        }
    }
}

// ========================================
// VoxelKey Implementation
// ========================================

VoxelKey::VoxelKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}

bool VoxelKey::operator==(const VoxelKey &other) const
{
    return x == other.x && y == other.y && z == other.z;
}

size_t VoxelKeyHash::operator()(const VoxelKey &key) const
{
    return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2);
}

// ========================================
// VoxelGrid Implementation
// ========================================

VoxelGrid::VoxelGrid(float voxel_size) : voxel_size_(voxel_size) {}

VoxelKey VoxelGrid::getVoxelKey(float x, float y, float z) const
{
    int vx = static_cast<int>(floor(x / voxel_size_));
    int vy = static_cast<int>(floor(y / voxel_size_));
    int vz = static_cast<int>(floor(z / voxel_size_));
    return VoxelKey(vx, vy, vz);
}

void VoxelGrid::insertPoint(float x, float y, float z, float distance)
{
    VoxelKey key = getVoxelKey(x, y, z);

    auto it = voxel_grid_.find(key);
    if (it != voxel_grid_.end())
    {
        it->second.insertPoint(x, y, z, distance);
    }
    else
    {
        float voxel_min_x = key.x * voxel_size_;
        float voxel_min_y = key.y * voxel_size_;
        float voxel_min_z = key.z * voxel_size_;

        ParentVoxel parent_voxel(voxel_min_x, voxel_min_y, voxel_min_z, voxel_size_);
        parent_voxel.insertPoint(x, y, z, distance);
        voxel_grid_[key] = parent_voxel;
    }
}

std::vector<VoxelPoint> VoxelGrid::getPoints() const
{
    std::vector<VoxelPoint> points;
    size_t estimated_size = voxel_grid_.size() * 4;
    points.reserve(estimated_size);

    for (const auto &pair : voxel_grid_)
    {
        pair.second.collectPoints(points);
    }

    return points;
}

void VoxelGrid::clear()
{
    voxel_grid_.clear();
}

} // namespace hokuyo_go
