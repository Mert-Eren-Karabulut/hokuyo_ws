#ifndef HOKUYO_GO_VOXEL_GRID_H
#define HOKUYO_GO_VOXEL_GRID_H

#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

namespace hokuyo_go
{

// ========================================
// Voxel Structures
// ========================================

/// Global Voxel Grid Parameters
#define MAX_SUBVOXEL_LEVEL 2 // Maximum subdivision level for multi-resolution voxels

extern float SUBVOXEL_THRESHOLD; // Occupancy threshold to subdivide voxel further

// Forward declaration
struct SubVoxel;

// Voxel structure for global voxelization with arithmetic averaging
struct VoxelPoint
{
    float x, y, z;         // Current averaged position
    float sensor_distance; // Distance to sensor when point was acquired
    int count;             // Number of points accumulated
    int subdivision_level; // Subdivision level (0 = parent, 1+ = subvoxel levels)

    VoxelPoint();
    VoxelPoint(float x_, float y_, float z_, float dist, int level = 0);

    // Update position using simple arithmetic averaging
    void updatePosition(float new_x, float new_y, float new_z, float new_distance);

    // Get RGB color based on subdivision level
    void getColor(uint8_t &r, uint8_t &g, uint8_t &b) const;
};

// SubVoxel structure for hierarchical subdivision
struct SubVoxel
{
    VoxelPoint point_data;          // Averaged point data for this subvoxel
    std::vector<SubVoxel> children; // Child subvoxels (8 children for octree)
    int current_level;              // Current subdivision level
    bool is_subdivided;             // Whether this subvoxel has been subdivided
    float min_x, min_y, min_z;      // Bounding box min
    float max_x, max_y, max_z;      // Bounding box max

    SubVoxel();
    SubVoxel(float minx, float miny, float minz, float maxx, float maxy, float maxz, int level);

    // Check if a point is within this subvoxel's bounds
    bool containsPoint(float x, float y, float z) const;

    // Get the child index for a point (octree indexing: 0-7)
    int getChildIndex(float x, float y, float z) const;

    // Create 8 children subvoxels
    void subdivide();

    // Remove subdivision and consolidate points
    void unsubdivide();
};

// Parent voxel structure containing subvoxels
struct ParentVoxel
{
    std::vector<SubVoxel> subvoxels; // 8 subvoxels
    VoxelPoint averaged_point;       // Fallback averaged point if not subdivided
    bool is_subdivided;              // Whether this voxel is using subvoxels
    float min_x, min_y, min_z;       // Voxel bounds
    float max_x, max_y, max_z;
    int total_points; // Total points in this parent voxel

    ParentVoxel();
    ParentVoxel(float minx, float miny, float minz, float voxel_size);

    // Insert point and manage subdivision
    void insertPoint(float x, float y, float z, float distance);

    int getSubvoxelIndex(float x, float y, float z) const;
    void insertIntoSubvoxel(SubVoxel &subvox, float x, float y, float z, float distance);
    void evaluateSubdivision();
    void evaluateSubvoxelSubdivision(SubVoxel &subvox);
    void evaluateSubvoxelSubdivisionRecursive(SubVoxel &subvox, int parent_total);

    // Collect all leaf points for visualization
    void collectPoints(std::vector<VoxelPoint> &points) const;
    void collectSubvoxelPoints(const SubVoxel &subvox, std::vector<VoxelPoint> &points) const;
};

// Voxel key for hash map
struct VoxelKey
{
    int x, y, z;

    VoxelKey(int x_, int y_, int z_);

    bool operator==(const VoxelKey &other) const;
};

// Hash function for VoxelKey
struct VoxelKeyHash
{
    size_t operator()(const VoxelKey &key) const;
};

// ========================================
// Voxel Grid Manager
// ========================================

class VoxelGrid
{
public:
    VoxelGrid(float voxel_size);

    void insertPoint(float x, float y, float z, float distance);
    std::vector<VoxelPoint> getPoints() const;
    void clear();
    size_t size() const { return voxel_grid_.size(); }

private:
    float voxel_size_;
    std::unordered_map<VoxelKey, ParentVoxel, VoxelKeyHash> voxel_grid_;

    VoxelKey getVoxelKey(float x, float y, float z) const;
};

} // namespace hokuyo_go

#endif // HOKUYO_GO_VOXEL_GRID_H
