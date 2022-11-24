#include <pch.h>
#include <algorithm>
#include <cmath>
#include <set>

#include "helper_classes/tolerance.h"
#include "localsymmetry.h"

using namespace Symmetry;



// CLASS VARIABLES
// Vector of LAS points.
std::vector<Point<float>> LocalSymmetry::points = std::vector<Point<float>>();

// Object with the data about the voxel mesh.
VoxelMesh LocalSymmetry::voxelMesh = VoxelMesh();

// 3D vector of voxels.
std::vector<std::vector<std::vector<Voxel>>> LocalSymmetry::voxels;



// GETTERS
// Point vector getter.
const std::vector<Point<float>>& LocalSymmetry::getPoints() {
    return points;
}

// Voxel mesh object getter.
const VoxelMesh& LocalSymmetry::getVoxelMesh() {
    return voxelMesh;
}

// 3D voxel vector getter.
const std::vector<std::vector<std::vector<Voxel>>>& LocalSymmetry::getVoxels() {
    return voxels;
}



// CLASS METHODS
// Setting points.
void LocalSymmetry::setPoints(std::vector<Point<float>>& points) {
    LocalSymmetry::points = points;
    LocalSymmetry::voxelMesh = {};

    for (Point<float>& lasPoint : LocalSymmetry::points) {
        // Getting point coordinates.
        float x = lasPoint.getX();
        float y = lasPoint.getY();
        float z = lasPoint.getZ();

        // Finding minimum and maximum coordinates
        // of points to create the bounding box.
        if (x < voxelMesh.minX)
            voxelMesh.minX = x;
        if (x > voxelMesh.maxX)
            voxelMesh.maxX = x;
        if (y < voxelMesh.minY)
            voxelMesh.minY = y;
        if (y > voxelMesh.maxY)
            voxelMesh.maxY = y;
        if (z < voxelMesh.minZ)
            voxelMesh.minZ = z;
        if (z > voxelMesh.maxZ)
            voxelMesh.maxZ = z;
    }

    // Calculation of the difference between minimum
    // and maximum point coordinates.
    voxelMesh.deltaX = voxelMesh.maxX - voxelMesh.minX;
    voxelMesh.deltaY = voxelMesh.maxY - voxelMesh.minY;
    voxelMesh.deltaZ = voxelMesh.maxZ - voxelMesh.minZ;

    for (unsigned long long i = 0; i < points.size(); i++)  {
        LocalSymmetry::points[i].setX(LocalSymmetry::points[i].getX() - voxelMesh.minX);
        LocalSymmetry::points[i].setY(LocalSymmetry::points[i].getY() - voxelMesh.minY);
        LocalSymmetry::points[i].setZ(LocalSymmetry::points[i].getZ() - voxelMesh.minZ);
    }
    voxelMesh.minX = voxelMesh.minY = voxelMesh.minZ = 0;
    voxelMesh.maxX = voxelMesh.deltaX;
    voxelMesh.maxY = voxelMesh.deltaY;
    voxelMesh.maxZ = voxelMesh.deltaZ;
}

// Voxel mesh calculation by voxel side length.
void LocalSymmetry::calculateVoxelMeshByVoxelSideLength(const int userInput) {
    // Calculating X, Y, Z and total voxel count.
    const int x = (int)floor(voxelMesh.deltaX / userInput) + 1;  // X coordinate voxel count.
    const int y = (int)floor(voxelMesh.deltaY / userInput) + 1;  // Y coordinate voxel count.
    const int z = (int)floor(voxelMesh.deltaZ / userInput) + 1;  // Z coordinate voxel count.
    const int count = x * y * z;                                 // Total voxel count.

    // Saving the optimal values to the voxel mesh.
    voxelMesh.voxelCount = count;
    voxelMesh.voxelSideSize = userInput;
    voxelMesh.voxelX = x;
    voxelMesh.voxelY = y;
    voxelMesh.voxelZ = z;

    // Building a 3D voxel vector.
    buildVoxelVector();
}

// Maximum voxel count calculation.
void LocalSymmetry::calculateVoxelMeshByMaximumVoxelCount(const int userInput) {
    int bestVoxelCount = 0;
    int bestVoxelSideSize = 0;
    int bestVoxelX = -1;
    int bestVoxelY = -1;
    int bestVoxelZ = -1;

    // Fitting the voxel mesh by X.
    for (int i = 1; i < voxelMesh.deltaX; i++) {
        voxelMesh.voxelSideSize = (int)ceil((double)voxelMesh.deltaX / i);
        voxelMesh.voxelX = std::min(i, (int)ceil((double)voxelMesh.deltaX / voxelMesh.voxelSideSize)) + 1;
        voxelMesh.voxelY = (int)ceil((double)voxelMesh.deltaY / voxelMesh.voxelSideSize) + 1;
        voxelMesh.voxelZ = (int)ceil((double)voxelMesh.deltaZ / voxelMesh.voxelSideSize) + 1;
        voxelMesh.voxelCount = voxelMesh.voxelX * voxelMesh.voxelY * voxelMesh.voxelZ;

        // If the voxel count is larger than the
        // maximum input value, the search is over.
        if (voxelMesh.voxelCount > userInput) {
            break;
        }

        // If a better option is found, it is stored in the voxel mesh.
        if (voxelMesh.voxelCount > bestVoxelCount) {
            bestVoxelCount = voxelMesh.voxelCount;
            bestVoxelSideSize = voxelMesh.voxelSideSize;
            bestVoxelX = voxelMesh.voxelX;
            bestVoxelY = voxelMesh.voxelY;
            bestVoxelZ = voxelMesh.voxelZ;

            // Voxel edge size is minimum 1, therefore the search is over.
            if (voxelMesh.voxelSideSize == 1) {
                break;
            }
        }
    }

    // Fitting the voxel mesh by Y.
    for (int i = 1; i < voxelMesh.deltaY; i++) {
        voxelMesh.voxelSideSize = (int)ceil((double)voxelMesh.deltaY / i);
        voxelMesh.voxelX = (int)ceil((double)voxelMesh.deltaX / voxelMesh.voxelSideSize) + 1;
        voxelMesh.voxelY = std::min(i, (int)ceil((double)voxelMesh.deltaY / voxelMesh.voxelSideSize)) + 1;
        voxelMesh.voxelZ = (int)ceil((double)voxelMesh.deltaZ / voxelMesh.voxelSideSize) + 1;
        voxelMesh.voxelCount = voxelMesh.voxelX * voxelMesh.voxelY * voxelMesh.voxelZ;

        // If the voxel count is larger than the
        // maximum input value, the search is over.
        if (voxelMesh.voxelCount > userInput) {
            break;
        }

        // If a better option is found, it is stored in the voxel mesh.
        if (voxelMesh.voxelCount > bestVoxelCount) {
            bestVoxelCount = voxelMesh.voxelCount;
            bestVoxelSideSize = voxelMesh.voxelSideSize;
            bestVoxelX = voxelMesh.voxelX;
            bestVoxelY = voxelMesh.voxelY;
            bestVoxelZ = voxelMesh.voxelZ;

            // Voxel edge size is minimum 1, therefore the search is over.
            if (voxelMesh.voxelSideSize == 1) {
                break;
            }
        }
    }

    // Fitting the voxel mesh by Z.
    for (int i = 1; i < voxelMesh.deltaZ; i++) {
        voxelMesh.voxelSideSize = (int)ceil((double)voxelMesh.deltaZ / i);
        voxelMesh.voxelX = (int)ceil((double)voxelMesh.deltaX / voxelMesh.voxelSideSize) + 1;
        voxelMesh.voxelY = (int)ceil((double)voxelMesh.deltaY / voxelMesh.voxelSideSize) + 1;
        voxelMesh.voxelZ = std::min(i, (int)ceil((double)voxelMesh.deltaZ / voxelMesh.voxelSideSize)) + 1;
        voxelMesh.voxelCount = voxelMesh.voxelX * voxelMesh.voxelY * voxelMesh.voxelZ;

        // Voxel edge size is minimum 1, therefore the search is over.
        if (voxelMesh.voxelCount > userInput) {
            break;
        }

        // If a better option is found, it is stored in the voxel mesh.
        if (voxelMesh.voxelCount > bestVoxelCount) {
            bestVoxelCount = voxelMesh.voxelCount;
            bestVoxelSideSize = voxelMesh.voxelSideSize;
            bestVoxelX = voxelMesh.voxelX;
            bestVoxelY = voxelMesh.voxelY;
            bestVoxelZ = voxelMesh.voxelZ;

            // Voxel edge size is minimum 1, therefore the search is over.
            if (voxelMesh.voxelSideSize == 1) {
                break;
            }
        }
    }

    // If no solution is found, the whole scene is represented as 1 voxel.
    // Not nice for a user but nice for the algorithm :)
    if (bestVoxelCount == 0) {
        voxelMesh.voxelCount = 1;
        voxelMesh.voxelSideSize = (int)std::max({voxelMesh.deltaX, voxelMesh.deltaY, voxelMesh.deltaZ});
        voxelMesh.voxelX = 1;
        voxelMesh.voxelY = 1;
        voxelMesh.voxelZ = 1;
    }
    else {
        // Saving the optimal values to the voxel mesh.
        voxelMesh.voxelCount = bestVoxelCount;
        voxelMesh.voxelSideSize = bestVoxelSideSize;
        voxelMesh.voxelX = bestVoxelX;
        voxelMesh.voxelY = bestVoxelY;
        voxelMesh.voxelZ = bestVoxelZ;
    }

    // Building a 3D voxel vector.
    buildVoxelVector();
}

// Building a 3D voxel vector from a voxel mesh.
void LocalSymmetry::buildVoxelVector() {
    voxels.clear();  // Clearing a previous obsolete voxel vector.

    // Z dimension.
    for (int z = 0; z < voxelMesh.voxelZ; z++) {
        voxels.push_back(std::vector<std::vector<Voxel>>());  // Adding a new layer.

        // Y dimension.
        for (int y = 0; y < voxelMesh.voxelY; y++) {
            voxels[z].push_back(std::vector<Voxel>());  // Adding a new row in a layer.

            // X dimension.
            for (int x = 0; x < voxelMesh.voxelX; x++) {
                const int voxelX = (int)floor(voxelMesh.minX + (x * voxelMesh.voxelSideSize));  // X coordinate.
                const int voxelY = (int)floor(voxelMesh.minY + (y * voxelMesh.voxelSideSize));  // Y coordinate.
                const int voxelZ = (int)floor(voxelMesh.minZ + (z * voxelMesh.voxelSideSize));  // Z coordinate.

                voxels[z][y].push_back(Voxel(voxelX, voxelY, voxelZ, false, false));  // Adding a new voxel.
            }
        }
    }
}

// Removing small clusters of interesting voxels.
void LocalSymmetry::removeSmallInterestingClusters(const int minimumClusterSize, int& interestingCount) {
    // If minimum cluster size equals 1, no clusters have to be removed. Yaaayyy!
    if (minimumClusterSize == 1) {
        return;
    }

    std::vector<std::set<Voxel>> clusters = findInterestingClusters();  // Vector with clusters.

    // Removing clusters that contain too few voxels.
    for (unsigned long long i = 0; i < clusters.size(); i++) {
        if ((int)clusters[i].size() < minimumClusterSize) {
            for (const Voxel& v : clusters[i]) {
                voxels[v.getZ()][v.getY()][v.getX()].setInteresting(false);
                voxels[v.getZ()][v.getY()][v.getX()].setSuperInteresting(false);
                interestingCount--;
            }
        }
    }
}

// Searching interesting voxels according to points in the 3D voxel vector.
void LocalSymmetry::findInterestingVoxels(const int minClusterSize, int& superInteresting, int& interesting) {
    // Building a fresh voxel vector.
    buildVoxelVector();

    superInteresting = 0;  // Number of super-interesting voxels.
    interesting = 0;       // Number of interesting voxels.

    // Variables for X, Y and Z coordinates.
    int x = -1;
    int y = -1;
    int z = -1;

    // Adding all super-interesting voxels.
    for (unsigned long long i = 0; i < points.size(); i++) {
        x = (int)floor((points[i].getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
        y = (int)floor((points[i].getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
        z = (int)floor((points[i].getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);

        if (z == voxelMesh.voxelZ) {
            z--;
        }

        if (Tolerance::isInTolerance(x, (points[i].getX() - voxelMesh.minX) / voxelMesh.voxelSideSize, 0.0001) ||
            Tolerance::isInTolerance(y, (points[i].getY() - voxelMesh.minY) / voxelMesh.voxelSideSize, 0.0001)
        )
        {
            continue;
        }

        // Searching the voxel in the vector and setting
        // super-interesting (temporary) and interesting.
        if (!voxels[z][y][x].getSuperInteresting()) {
            voxels[z][y][x].setSuperInteresting(true);
            voxels[z][y][x].setInteresting(true);
            superInteresting++;
            interesting++;
        }
    }

    // Removing too small clusters.
    removeSmallInterestingClusters(minClusterSize, interesting);
    superInteresting = interesting;

    // Removing super-interesting voxels in the middle,
    // so that the whole planes are not searched.
    std::vector<std::vector<std::vector<Voxel>>> tempVoxels(voxels);
    for (unsigned long long z = 0; z < voxels.size(); z++) {
        for (unsigned long long y = 1; y < voxels[z].size() - 1; y++) {
            for (unsigned long long x = 1; x < voxels[z][y].size() - 1; x++) {
                if (voxels[z][y].size() >= 3 &&
                    voxels[z].size() >= 3 &&
                    tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y][x - 1].getInteresting() &&
                    tempVoxels[z][y + 1][x].getInteresting() &&
                    tempVoxels[z][y + 1][x + 1].getInteresting() &&
                    tempVoxels[z][y - 1][x].getInteresting()
                )
                {
                    voxels[z][y][x].setSuperInteresting(false);
                    superInteresting--;
                }
            }
        }
    }

    // Removing super-interesting voxels in the middle,
    // so that the whole planes are not searched.
    for (unsigned long long z = 1; z < voxels.size() - 1; z++) {
        for (unsigned long long y = 0; y < voxels[z].size(); y++) {
            for (unsigned long long x = 1; x < voxels[z][y].size() - 1; x++) {
                if (
                    tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y][x - 1].getInteresting() &&
                    tempVoxels[z + 1][y][x].getInteresting() &&
                    tempVoxels[z][y][x + 1].getInteresting() &&
                    tempVoxels[z - 1][y][x].getInteresting()
                )
                {
                    voxels[z][y][x].setSuperInteresting(false);
                    superInteresting--;
                }
            }
        }
    }

    // Removing super-interesting voxels in the middle,
    // so that the whole planes are not searched.
    for (unsigned long long z = 1; z < voxels.size() - 1; z++) {
        for (unsigned long long y = 1; y < voxels[z].size() - 1; y++) {
            for (unsigned long long x = 0; x < voxels[z][y].size(); x++) {
                if (
                    tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y - 1][x].getInteresting() &&
                    tempVoxels[z + 1][y][x].getInteresting() &&
                    tempVoxels[z][y + 1][x].getInteresting() &&
                    tempVoxels[z - 1][y][x].getInteresting()
                )
                {
                    voxels[z][y][x].setSuperInteresting(false);
                    superInteresting--;
                }
            }
        }
    }

    // Removing super-interesting voxels in the middle,
    // so that the whole planes are not searched.
    for (unsigned long long z = 1; z < voxels.size() - 1; z++) {
        for (unsigned long long y = 1; y < voxels[z].size() - 1; y++) {
            for (unsigned long long x = 1; x < voxels[z][y].size() - 1; x++) {
                if (
                    (tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y][x - 1].getInteresting() &&
                    tempVoxels[z][y][x + 1].getInteresting() &&
                    tempVoxels[z + 1][y + 1][x].getInteresting() &&
                    tempVoxels[z - 1][y - 1][x].getInteresting())
                    ||
                    (tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y][x - 1].getInteresting() &&
                    tempVoxels[z][y][x + 1].getInteresting() &&
                    tempVoxels[z - 1][y + 1][x].getInteresting() &&
                    tempVoxels[z + 1][y - 1][x].getInteresting())
                    ||
                    (tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y - 1][x].getInteresting() &&
                    tempVoxels[z - 1][y][x + 1].getInteresting() &&
                    tempVoxels[z][y - 1][x].getInteresting() &&
                    tempVoxels[z + 1][y][x - 1].getInteresting())
                    ||
                    (tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z][y - 1][x].getInteresting() &&
                    tempVoxels[z + 1][y][x + 1].getInteresting() &&
                    tempVoxels[z][y - 1][x].getInteresting() &&
                    tempVoxels[z - 1][y][x - 1].getInteresting())
                    ||
                    (tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z + 1][y][x].getInteresting() &&
                    tempVoxels[z - 1][y][x].getInteresting() &&
                    tempVoxels[z][y + 1][x - 1].getInteresting() &&
                    tempVoxels[z][y - 1][x + 1].getInteresting())
                    ||
                    (tempVoxels[z][y][x].getInteresting() &&
                    tempVoxels[z + 1][y][x].getInteresting() &&
                    tempVoxels[z - 1][y][x].getInteresting() &&
                    tempVoxels[z][y - 1][x - 1].getInteresting() &&
                    tempVoxels[z][y + 1][x + 1].getInteresting())
                )
                {
                    voxels[z][y][x].setSuperInteresting(false);
                    superInteresting--;
                }
            }
        }
    }
}

// Splitting a line segment vector into parts, where the lenghts of line segments is below the tolerance.
std::vector<std::vector<LineSegment>> LocalSymmetry::splitLineSegmentVectorIntoParts(
    const std::vector<LineSegment>& lineSegments,
    const double tolerance
)
{
    // Creating a vector and pushing a first part.
    std::vector<std::vector<LineSegment>> splitLineSegments;
    splitLineSegments.push_back(std::vector<LineSegment>());

    // Iterating through all line segments.
    for (unsigned long long i = 0; i < lineSegments.size(); i++) {
        // If no elements in vector, a first line segment is added.
        if (splitLineSegments.back().size() == 0) {
            splitLineSegments.back().push_back(lineSegments[i]);
        }
        // If a line segment length is inside of the allowed tolerance,
        // it is added to the same part as the previous one.
        else if (
            Tolerance::isInTolerance(
                splitLineSegments.back()[0].getLength(),
                lineSegments[i].getLength(),
                tolerance
            )
        )
        {
            splitLineSegments.back().push_back(lineSegments[i]);
        }
        // If a line segment length is outside of the allowed tolerance,
        // a new part is created, line segment is added to the new part.
        else {
            // If the last part contains only one line
            // segment, there will be no symmetry.
            if (splitLineSegments.back().size() == 1) {
                splitLineSegments.pop_back();
            }

            splitLineSegments.push_back(std::vector<LineSegment>());
            splitLineSegments.back().push_back(lineSegments[i]);
        }
    }

    // If the last part contains only one line
    // segment, there will be no symmetry.
    if (splitLineSegments.back().size() == 1) {
        splitLineSegments.pop_back();
    }

    return splitLineSegments;
}

// Getting a normalized 3D voxel vector (voxel edge size equals 1).
std::vector<std::vector<std::vector<Voxel>>> LocalSymmetry::getNormalisedVoxelVector() {
    // Vector for storing normalized voxels.
    std::vector<std::vector<std::vector<Voxel>>> normalisedVoxelVector;

    // Z dimension.
    for (int z = 0; z < voxelMesh.voxelZ; z++) {
        normalisedVoxelVector.push_back(std::vector<std::vector<Voxel>>());

        // Y dimension.
        for (int y = 0; y < voxelMesh.voxelY; y++) {
            normalisedVoxelVector[z].push_back(std::vector<Voxel>());

            // X dimension.
            for (int x = 0; x < voxelMesh.voxelX; x++) {
                normalisedVoxelVector[z][y].push_back(
                    Voxel(x, y, z, voxels[z][y][x].getSuperInteresting(), voxels[z][y][x].getInteresting())
                );
            }
        }
    }

    return normalisedVoxelVector;
}

// Clustering recursive step.
void LocalSymmetry::interestingClusterRecursiveStep(std::set<Voxel>& cluster, std::vector<std::vector<std::vector<Voxel>>>& voxelVector, const int x, const int y, const int z) {
    // If the voxel coordinates lie outside of the voxel mesh or the
    // voxel has been already checked, the recursion unfolds.
    if (x < 0 || x >= voxelMesh.voxelX ||
        y < 0 || y >= voxelMesh.voxelY ||
        z < 0 || z >= voxelMesh.voxelZ ||
        voxelVector[z][y][x].getChecked()
    )
    {
        return;
    }

    // Setting the current voxel to checked.
    voxelVector[z][y][x].setChecked(true);

    // If the current voxel is not interesting, the recursion starts to unfold.
    if (!voxelVector[z][y][x].getInteresting()) {
        return;
    }

    // Inserting the current voxel to the cluster.
    cluster.insert(Voxel(x, y, z, true));

    // 26 recursive steps to for depth-first search of the neighborhood.
    // 9 in the upper layer, 8 (all but current) in the current layer, 9 in the lower layer.
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y - 1, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y - 1, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y - 1, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y + 0, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y + 0, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y + 0, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y + 1, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y + 1, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y + 1, z + 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y - 1, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y - 1, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y - 1, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y + 0, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y + 0, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y + 0, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y + 1, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y + 1, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y + 1, z + 0);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y - 1, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y - 1, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y - 1, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y + 0, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y + 0, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y + 0, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x - 1, y + 1, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 0, y + 1, z - 1);
    interestingClusterRecursiveStep(cluster, voxelVector, x + 1, y + 1, z - 1);
}

// Finding clusters of symmetry voxels in symmetry.
std::vector<std::set<Voxel>> LocalSymmetry::findInterestingClusters() {
    std::vector<std::set<Voxel>> clusters;

    // Getting the 3D voxel vector.
    auto voxels = getNormalisedVoxelVector();

    // Searching for clusters in each voxel in symmetry (depth-first-search).
    for (size_t z = 0; z < voxels.size(); z++) {
        for (size_t y = 0; y < voxels[z].size(); y++) {
            for (size_t x = 0; x < voxels[z][y].size(); x++) {
                // If the current voxel has not been checked and is in symmetry, a new cluster has been found.
                if (!voxels[z][y][x].getChecked() && voxels[z][y][x].getInteresting()) {
                    std::set<Voxel> cluster;                                    // Creating a new cluster.
                    interestingClusterRecursiveStep(cluster, voxels, x, y, z);  // Recursively search of the current cluster voxels.
                    clusters.push_back(cluster);                                // Adding a new cluster to the list.
                }
            }
        }
    }

    return clusters;
}

// Getting line segments between all pairs of points (fully connected graph).
std::vector<LineSegment> LocalSymmetry::calculateLineSegmentsBetweenPoints(
    std::vector<Point<float>> points,
    const double tolerance)
{
    std::vector<LineSegment> lineSegments;

    // Calculation of every pair of points.
    for (unsigned long long i = 0; i < points.size() - 1; i++) {
        for (unsigned long long j = i + 1; j < points.size(); j++) {
            // Cycles are not allowed in graph.
            // Line segments between points with greater Z coordinate
            // difference than tolerance are ignored.
            if (i != j && Tolerance::isInTolerance(points[i].getZ(), points[j].getZ(), tolerance)) {
                LineSegment ls(points[i], points[j]);
                auto percent = LocalSymmetry::calculateInterestingPercentInLineSegment(ls);
                ls.setFullQuotient(percent);

//                if (percent >= 0.8) {
                    lineSegments.push_back(ls);
//                }
            }
        }
    }

    // Sorting all line segments by length.
    std::sort(
        lineSegments.begin(),
        lineSegments.end(),
        [](LineSegment a, LineSegment b) { return a.getLength() < b.getLength(); }
    );

    return lineSegments;
}

// Calculating the percent of interesting line segment sections.
float LocalSymmetry::calculateInterestingPercentInLineSegment(const LineSegment& lineSegment) {
    // Calculating the number of segments.
    int interestingSegments = 0;
    const int segments = (int)(10 * lineSegment.getLength());
    auto& voxels = LocalSymmetry::getVoxels();

    // Calculating the vector from P1 to P2.
    Point<float> p1 = lineSegment.getP1();
    Point<float> p2 = lineSegment.getP2();
    auto difference = p2 - p1;

    // Marching on the line segment and calculating
    // interesting line segment sections.
    for (int i = 0; i < segments; i++) {
        double factor = (double)i / segments;
        Vector3d p = Vector3d(
            p1.getX() + factor * difference.getX(),
            p1.getY() + factor * difference.getY(),
            p1.getZ() + factor * difference.getZ()
        );

        // Calculating a voxel position.
        const int x = (int)floor((p.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
        const int y = (int)floor((p.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
        const int z = (int)floor((p.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);

        // If a voxel is interesting, the number of
        // interesting sections is incremented.
        if (voxels[z][y][x].getInteresting()) {
            interestingSegments++;
        }
    }

    return (float)interestingSegments / (float)segments;
}

