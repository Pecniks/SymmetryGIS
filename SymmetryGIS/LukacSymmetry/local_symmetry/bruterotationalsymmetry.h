#ifndef BRUTEROTATIONALSYMMETRY_H
#define BRUTEROTATIONALSYMMETRY_H

#include <vector>

#include "helper_classes/point.h"
#include "helper_classes/positionfromplane.h"
#include "helper_classes/voxel.h"
#include "localsymmetry.h"


namespace Symmetry {
    // Class for rotational symmetries.
    class BruteRotationalSymmetry : public LocalSymmetry {
    private:
        // OBJECT VARIABLES
        std::vector<Voxel> voxels;  // Vector of pointers to voxels, included in the symmetry.
        Point<double> symmetryAxis;   // Voxel, where the symmetry axis is.
        uint8_t rotation;             // Rotation level (1, 2, ..., n).
        uint32_t upperLayer;          // Index of the starting (upper) layer.
        uint32_t lowerLayer;          // Index of the ending (lower) layer.

        // CLASS VARIABLES
        static inline std::vector<float> symmetryAngles;              // Angles of the rotational symmetry.
        static inline std::vector<std::vector<bool>> geometrySearch;  // Vector for voxel search in 2D (layer).
        static inline int32_t geometrySearchSizeX;                    // Dimension X size in the voxel field.
        static inline int32_t geometrySearchSizeY;                    // Dimension Y size in the voxel field.

        // PRIVATE OBJECT METHODS
        std::vector<BruteRotationalSymmetry> getRotationalSymmetriesAroundCenterPoint(std::vector<std::vector<Voxel>>& field, Point<double> centerPoint, const uint16_t maxRadius, int32_t pixelX, int32_t pixelY);  // Function that increments the radius of the circumference around the center point with the step of 1. In every step it selects all interesting voxels the circumference overlaps with and calls the function FindRotationalSymmetries. In the end a union of all interesting voxels from obtained symmetries with different radii is made. The function returns a list of rotational symmetries (2 through 16), where each rotational symmetry is present once at most.
        std::vector<BruteRotationalSymmetry> mergeSymmetries(std::vector<BruteRotationalSymmetry>& symmetries);  // Merging symmetries by layers. Symmetries with the same search axis and rotation level are merged.
        std::vector<BruteRotationalSymmetry> getRotationalSymmetriesOnRadius(Point<double> centerPoint, const std::vector<Voxel>& interestingVoxels);  // A main function for searching symmetries gets a center point and a list of interesting voxels. Symmetries are searched for only in the set of interesting voxels that are on the same radius from the center point (or inside the same interval).
    public:
        // CONSTRUCTOR AND DESTRUCTOR
        BruteRotationalSymmetry();   // Default constructor.
        BruteRotationalSymmetry(const Point<double>& symmetryAxis, const uint8_t& rotation, const uint32_t& upperLayer, const uint32_t& lowerLayer);  // Constructor with all parameters.

        // OVERLOADED OPERATORS
        BruteRotationalSymmetry operator & (const BruteRotationalSymmetry& s) const;  // Operator & is used for merging two symmetries.
        BruteRotationalSymmetry operator &= (BruteRotationalSymmetry& s);             // Operator &= is used for merging the current symmetry with another.

        // GETTERS AND SETTERS
        int getVoxelCount() const;                         // Getting voxel count.
        std::vector<Voxel>& getVoxels();                 // Getting all voxels.
        Voxel getVoxel(int index) const;                  // Getting a voxel at a certain index.
        void addVoxel(Voxel v);                           // Adding a voxel to the vector.
        Point<double> getSymmetryAxis() const;             // Getting a symmetry axis.
        void setSymmetryAxis(Point<double> symmetryAxis);  // Setting a symmetry axis.
        uint8_t getRotation() const;                       // Getting a rotation.
        void setRotation(uint8_t rotation);                // Setting a rotation.
        uint32_t getUpperLayer() const;                    // Getting an upper layer.
        void setUpperLayer(uint32_t upperLayer);           // Setting an upper layer.
        uint32_t getLowerLayer() const;                    // Getting a lower layer.
        void setLowerLayer(uint32_t lowerLayer);           // Setting a lower layer.

        // CLASS METHODS
        static void setGeometrySearch(int32_t x, int32_t y);    // Symmetry search class initialization method.
        static void clearGeometrySearch(int32_t x, int32_t y);  // Cleaning the history of geometry search.
        static void setSymmetryAngles();                        // Calculation of angles of rotational symmetries (interval [1, 16]).
        static void getDistanceRanges(Point<double> centerPoint, Voxel v, double& minD, double& maxD);  // Calculation of the minimum and the maximum distance from the center point to the voxel (in 2D).
        static float getAngle(const Point<double> a, const Point<double> b);  // Getting the angle between two vectors (represented as Points).
        static void getAngleRanges(const Point<double>& centerPoint, Voxel v, float& minAngle, float& maxAngle);  // Calculation of the minimum and the maximum angle between horizontal vector and the vectors from the center point to the vertices of the voxel (in 2D).

        // OBJECT METHODS
        std::vector<BruteRotationalSymmetry> getRotationalSymmetries(std::vector<std::vector<std::vector<Voxel>>> voxels, VoxelMesh& voxelMesh);  // Finding rotational symmetries.
        std::vector<PositionFromPlane> calculatePositionsFromAxis(const Point<double>* axis);  // Calculating the point positions according to the axis.
    };
};

#endif // BRUTEROTATIONALSYMMETRY_H
