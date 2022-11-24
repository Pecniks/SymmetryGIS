#ifndef LOCALSYMMETRY_H
#define LOCALSYMMETRY_H

#include <set>
#include <string>
#include <vector>

#include "helper_classes/linesegment.h"
#include "helper_classes/point.h"
#include "helper_classes/voxel.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif


namespace Symmetry {
    // Class for local symmetries that contains methods for
    // both reflection symmetries and rotational symmetries.
    class LocalSymmetry {
    protected:
        // CLASS VARIABLES
        static std::vector<Point<float>> points;                     // Vector of LAS points.
        static VoxelMesh voxelMesh;                                  // Object with the data about the voxel mesh.
        static std::vector<std::vector<std::vector<Voxel>>> voxels;  // 3D vector of voxels.

    public:
        // GETTERS
        static const std::vector<Point<float>>& getPoints();                     // Point vector getter.
        static const VoxelMesh& getVoxelMesh();                                  // Voxel mesh object getter.
        static const std::vector<std::vector<std::vector<Voxel>>>& getVoxels();  // 3D voxel vector getter.

        // CLASS METHODS
        static void setPoints(std::vector<Point<float>>& points);                                              // Setting points.
        static void calculateVoxelMeshByVoxelSideLength(const int userInput);                                  // Voxel mesh calculation by voxel side length.
        static void calculateVoxelMeshByMaximumVoxelCount(const int userInput);                                // Voxel mesh calculation by maximum voxel count.
        static void buildVoxelVector();                                                                        // Building a 3D voxel vector from a voxel mesh.
        static void removeSmallInterestingClusters(const int minimumClusterSize, int& interestingCount);       // Removing small clusters of interesting voxels.
        static void findInterestingVoxels(const int minClusterSize, int& superInteresting, int& interesting);  // Searching super-interesting voxels according to points in the 3D voxel vector.
        static std::vector<std::vector<LineSegment>> splitLineSegmentVectorIntoParts(const std::vector<LineSegment>& lineSegments, const double tolerance);  // Splitting a line segment vector into parts, where the lenghts of line segments is below the tolerance.
        static std::vector<std::vector<std::vector<Voxel>>> getNormalisedVoxelVector();                        // Getting a normalized 3D voxel vector (voxel edge size equals 1).
        static void interestingClusterRecursiveStep(std::set<Voxel>& cluster, std::vector<std::vector<std::vector<Voxel>>>& voxelVector, const int x, const int y, const int z);  // Clustering recursive step.
        static std::vector<std::set<Voxel>> findInterestingClusters();  // Finding clusters of symmetry voxels in symmetry.
        static std::vector<LineSegment> calculateLineSegmentsBetweenPoints(std::vector<Point<float>> points, const double tolerance);    // Getting line segments between all pairs of points (fully connected graph).
        static float calculateInterestingPercentInLineSegment(const LineSegment& lineSegment);  // Calculating the percent of interesting line segment sections.
    };
};

#endif // LOCALSYMMETRY_H
