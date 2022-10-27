#ifndef VOXEL_H
#define VOXEL_H

#include <climits>
#include <vector>

#include "point.h"


namespace Symmetry {
    // Declaration of VoxelMesh.
    struct VoxelMesh;

    // Voxel for discretization in the 3D space.
    class Voxel {
    private:
        // OBJECT VARIABLES
        int x;                               // X coordinate.
        int y;                               // Y coordinate.
        int z;                               // Z coordinate.
        bool superInteresting;               // True if the voxel contains at least one point and is not surrounded with other interesting voxels.
        bool interesting;                    // True if the voxel contains at least one point and is not surrounded with other interesting voxels.
        bool checked;                        // True if the voxel has been checked during the geometry search.
        bool inSymmetry;                     // True if the voxel is a part of a symmetry.
    public:
        // CONSTRUCTORS
        Voxel();                                      // Default constructor without parameters.
        Voxel(int x, int y, int z);                   // Constructor with all three coordinates.
        Voxel(int x, int y, int z, bool si, bool i);  // Constructor with all three coordinates and booleans for a super-interesting and an interesting voxel.
        Voxel(int x, int y, int z, bool is);          // Constructor with all three coordinates and a boolean for in-symmetry.
        Voxel(Point<int>& p, bool i);                 // Copy constructor with Point<T> and a boolean for an interesting voxel.
        Voxel(Point<float>& p);                       // Copy constructor with Point<float>.

        // GETTERS AND SETTERS
        int getX() const;                                  // Getting X coordinate.
        void setX(const int x);                            // Setting X coordinate.
        int getY() const;                                  // Getting Y coordinate.
        void setY(const int y);                            // Setting Y coordinate.
        int getZ() const;                                  // Getting Z coordinate.
        void setZ(const int z);                            // Setting Z coordinate.
        bool getSuperInteresting() const;                  // Getting super-interesting property.
        void setSuperInteresting(const bool interesting);  // Setting super-interesting property.
        bool getInteresting() const;                       // Getting interesting property.
        void setInteresting(const bool interesting);       // Setting interesting property.
        bool getChecked() const;                           // Getting checked property.
        void setChecked(const bool checked);               // Setting checked property.
        bool getInSymmetry() const;                        // Getting in symmetry property.
        void setInSymmetry(const bool inSymmetry);         // Setting in symmetry property.

        // CLASS METHODS
        static double distance(Voxel v1, Voxel v2);                           // Calculation of the Euclidian distance between two voxels.
        static bool isBrightNeighbor(const int x, const int y, const int z);  // Getting the data whether the voxel should be painted brightly.
        static int getLayerInVoxelMesh(const int z, const VoxelMesh& vm);     // Getting the layer of the voxel mesh according to the Z coordinate.
        static std::vector<Point<float>> getInterestingPointsFromVoxels(std::vector<std::vector<std::vector<Voxel>>> voxels, const float voxelEdge, const bool mustBeSuperInteresting = true);  // Returning interesting points from voxel vector.

        // OBJECT METHODS
        bool operator == (const Voxel v) const;  // Checking whether the two voxels have the same coordinates.
        bool operator < (Voxel v) const;         // Operator < serves for the sort method as the comparator.
    };


    // Structure for dividing the voxels into a mesh.
    struct VoxelMesh {
        int voxelCount = -1;          // Number of voxels.
        int voxelSideSize = 0;        // Length of a voxel edge.
        int voxelX = -1;              // Number of voxels by X.
        int voxelY = -1;              // Number of voxels by Y.
        int voxelZ = -1;              // Number of voxels by Z.
        float minX = (float)INT_MAX;  // Minimum X.
        float maxX = (float)INT_MIN;  // Maximum X.
        float minY = (float)INT_MAX;  // Minimum Y.
        float maxY = (float)INT_MIN;  // Maximum Y.
        float minZ = (float)INT_MAX;  // Minimum Z.
        float maxZ = (float)INT_MIN;  // Maximum Z.
        float deltaX = -1.0;          // Distance between the maximum and the minimum X coordinate.
        float deltaY = -1.0;          // Distance between the maximum and the minimum Y coordinate.
        float deltaZ = -1.0;          // Distance between the maximum and the minimum Z coordinate.
    };
};


#endif // VOXEL_H
