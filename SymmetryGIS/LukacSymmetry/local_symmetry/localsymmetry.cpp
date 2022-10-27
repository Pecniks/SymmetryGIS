#include <pch.h>
#include <set>

#include "LAS/File.h"
#include "localsymmetry.h"

using namespace Symmetry;


// CLASS VARIABLES
// Vector of LAS points.
std::vector<Point<float>> LocalSymmetry::points = std::vector<Point<float>>();

// Object with the data about the voxel mesh.
VoxelMesh LocalSymmetry::voxelMesh = VoxelMesh();

// 3D vector of voxels.
std::vector<std::vector<std::vector<Voxel>>> LocalSymmetry::LocalSymmetry::voxels;



// GETTERS
// Point vector getter.
const std::vector<Point<float>>& LocalSymmetry::LocalSymmetry::getPoints() {
    return points;
}

// Voxel mesh object getter.
const VoxelMesh& LocalSymmetry::LocalSymmetry::getVoxelMesh() {
    return voxelMesh;
}

// 3D voxel vector getter.
const std::vector<std::vector<std::vector<Voxel>>>& LocalSymmetry::LocalSymmetry::getVoxels() {
    return voxels;
}



// CLASS METHODS
// Reading points from a LAS file.
void LocalSymmetry::readPoints(std::string path) {
    LAS::File las = LAS::File(path);       // Opening a LAS file with the laslib library.
    points = std::vector<Point<float>>();  // Resetting previous LAS points.
    voxelMesh = {};                        // Resetting previous voxel mesh.

    // Reading point by point and finding extreme points.
    for (auto lasPoint : las) {
        // Retreiving point coordinates given in centimeters,
        // therefore we convert them to meters and create new points.
        float x = lasPoint.X() / 100.0f;
        float y = lasPoint.Y() / 100.0f;
        float z = lasPoint.Z() / 100.0f;
        points.push_back(Point<float>(x, y, z));

        // Finding minimum and maximum coordinates
        // of coordinates to create the bounding box.
        // X coordinate.
        if (x < voxelMesh.minX) {
            voxelMesh.minX = x;
        }
        else if (x > voxelMesh.maxX) {
            voxelMesh.maxX = x;
        }

        // Y coordinate.
        if (y < voxelMesh.minY) {
            voxelMesh.minY = y;
        }
        else if (y > voxelMesh.maxY) {
            voxelMesh.maxY = y;
        }

        // Z coordinate.
        if (z < voxelMesh.minZ) {
            voxelMesh.minZ = z;
        }
        else if (z > voxelMesh.maxZ) {
            voxelMesh.maxZ = z;
        }
    }

    // Calculation of the difference between minimum and maximum
    // point coordinates (alternatively called delta).
    voxelMesh.deltaX = voxelMesh.maxX - voxelMesh.minX;
    voxelMesh.deltaY = voxelMesh.maxY - voxelMesh.minY;
    voxelMesh.deltaZ = voxelMesh.maxZ - voxelMesh.minZ;

    for (unsigned long long i = 0; i < points.size(); i++)  {
        points[i].setX(points[i].getX() - voxelMesh.minX);
        points[i].setY(points[i].getY() - voxelMesh.minY);
        points[i].setZ(points[i].getZ() - voxelMesh.minZ);
    }

    voxelMesh.minX = voxelMesh.minY = voxelMesh.minZ = 0;
    voxelMesh.maxX = voxelMesh.deltaX;
    voxelMesh.maxY = voxelMesh.deltaY;
    voxelMesh.maxZ = voxelMesh.deltaZ;

    /******************************************************************************************/
    /******************************************************************************************/
    /******************************************************************************************/
    /******************************************************************************************/

//    // Reading a LAS file.
//    points = {
//        Point<float>(0, 0, 0),
//        Point<float>(2, 0, 0),
//        Point<float>(4, 0, 0),
//        Point<float>(0, 2, 0),
//        Point<float>(2, 2, 0),
//        Point<float>(4, 2, 0),
//        Point<float>(0, 4, 0),
//        Point<float>(2, 4, 0),
//        Point<float>(4, 4, 0),

//        Point<float>(20, 0, 0),
//        Point<float>(18, 0, 0),
//        Point<float>(16, 0, 0),
//        Point<float>(20, 2, 0),
//        Point<float>(18, 2, 0),
//        Point<float>(16, 2, 0),
//        Point<float>(20, 4, 0),
//        Point<float>(18, 4, 0),
//        Point<float>(16, 4, 0),

//        Point<float>(2, 0, 3),
//        Point<float>(4, 0, 3),
//        Point<float>(2, 2, 3),
//        Point<float>(4, 2, 3),
//        Point<float>(2, 4, 3),
//        Point<float>(4, 4, 3),

//        Point<float>(4, 0, 4),
//        Point<float>(4, 2, 4),
//        Point<float>(4, 4, 4),

//        Point<float>(4, 0, 6),
//        Point<float>(4, 8, 6),
//        Point<float>(4, 16, 6),

//        Point<float>(12, 2.5, 0),
//    };

//    for (Point<float>& lasPoint : points) {
//        // Getting point coordinates.
//        float x = lasPoint.getX();
//        float y = lasPoint.getY();
//        float z = lasPoint.getZ();

//        // Finding minimum and maximum coordinates
//        // of points to create the bounding box.
//        if (x < voxelMesh.minX)
//            voxelMesh.minX = x;
//        if (x > voxelMesh.maxX)
//            voxelMesh.maxX = x;
//        if (y < voxelMesh.minY)
//            voxelMesh.minY = y;
//        if (y > voxelMesh.maxY)
//            voxelMesh.maxY = y;
//        if (z < voxelMesh.minZ)
//            voxelMesh.minZ = z;
//        if (z > voxelMesh.maxZ)
//            voxelMesh.maxZ = z;
//    }

//    // Calculation of the difference between minimum
//    // and maximum point coordinates.
//    voxelMesh.deltaX = voxelMesh.maxX - voxelMesh.minX;
//    voxelMesh.deltaY = voxelMesh.maxY - voxelMesh.minY;
//    voxelMesh.deltaZ = voxelMesh.maxZ - voxelMesh.minZ;

//    for (unsigned long long i = 0; i < points.size(); i++)  {
//        points[i].setX(points[i].getX() - voxelMesh.minX);
//        points[i].setY(points[i].getY() - voxelMesh.minY);
//        points[i].setZ(points[i].getZ() - voxelMesh.minZ);
//    }
//    voxelMesh.minX = voxelMesh.minY = voxelMesh.minZ = 0;
//    voxelMesh.maxX = voxelMesh.deltaX;
//    voxelMesh.maxY = voxelMesh.deltaY;
//    voxelMesh.maxZ = voxelMesh.deltaZ;
}

// Voxel mesh calculation by voxel side length.
void LocalSymmetry::calculateVoxelMeshByVoxelSideLength(const int userInput) {
    // Calculating X, Y, Z and total voxel count.
    const int x = (int)ceil(voxelMesh.deltaX / userInput);  // X coordinate voxel count.
    const int y = (int)ceil(voxelMesh.deltaY / userInput);  // Y coordinate voxel count.
    const int z = (int)ceil(voxelMesh.deltaZ / userInput);  // Z coordinate voxel count.
    const int count = x * y * z;                            // Total voxel count.

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
void LocalSymmetry::LocalSymmetry::calculateVoxelMeshByMaximumVoxelCount(const int userInput) {
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
    voxels = std::vector<std::vector<std::vector<Voxel>>>();  // Clearing a previous obsolete voxel vector.

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

    std::vector<std::set<Voxel>> clusters;  // Vector with clusters.

    // Searching for clusters in the voxel mesh.
    for (int z = 0; z < voxelMesh.voxelZ; z++) {
        for (int y = 0; y < voxelMesh.voxelY; y++) {
            for (int x = 0; x < voxelMesh.voxelX; x++) {
                if (voxels[z][y][x].getInteresting()) {
                    unsigned long long cluster = -1;
                    for (unsigned long long i = 0; i < clusters.size(); i++) {
                        // Upper layer.
                        if (z != voxelMesh.voxelZ - 1) {
                            // Upper row.
                            if (y != voxelMesh.voxelY - 1) {
                                // Upper left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y + 1, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Upper voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y + 1, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Upper right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y + 1, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }

                            // Middle row.
                            {
                                // Middle left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Middle voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Middle right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }

                            // Lower row.
                            if (y != 0) {
                                // Lower left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y - 1, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Lower voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y - 1, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Lower right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y - 1, z + 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }
                        }

                        // Current layer.
                        {
                            // Upper row.
                            if (y != voxelMesh.voxelY - 1) {
                                // Upper left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y + 1, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Upper voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y + 1, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Upper right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y + 1, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }

                            // Middle row.
                            {
                                // Middle left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Middle right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }

                            // Lower row.
                            if (y != 0) {
                                // Lower left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y - 1, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Lower voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y - 1, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Lower right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y - 1, z)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }
                        }

                        // Lower layer.
                        if (z != 0) {
                            // Upper row.
                            if (y != voxelMesh.voxelY - 1) {
                                // Upper left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y + 1, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Upper voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y + 1, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Upper right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y + 1, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }

                            // Middle row.
                            {
                                // Middle left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Middle voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Middle right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }

                            // Lower row.
                            if (y != 0) {
                                // Lower left voxel.
                                if (x != 0) {
                                    if (clusters[i].find(Voxel(x - 1, y - 1, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Lower voxel.
                                {
                                    if (clusters[i].find(Voxel(x, y - 1, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                                // Lower right voxel.
                                if (x != voxelMesh.voxelX - 1) {
                                    if (clusters[i].find(Voxel(x + 1, y - 1, z - 1)) != clusters[i].end()) {
                                        cluster = i;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    // If there is no existent cluster where our voxel would belong, a new cluster is added.
                    if (cluster < 0) {
                        clusters.push_back(std::set<Voxel>());
                        clusters.back().insert(Voxel(x, y, z));
                    }
                    // If there is an already existent cluster, the voxel is added to that one.
                    else {
                        clusters[cluster].insert(Voxel(x, y, z));
                    }
                }
            }
        }
    }

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

// Getting a normalized 3D voxel vector (voxel edge size equals 1).
std::vector<std::vector<std::vector<Voxel>>> LocalSymmetry::LocalSymmetry::getNormalisedVoxelVector() {
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
