#include <pch.h>
#include "voxel.h"
#include "point.h"

using namespace Symmetry;



// CONSTRUCTORS
// Default constructor without parameters.
Voxel::Voxel() :
    x(0),
    y(0),
    z(0),
    superInteresting(false),
    interesting(false),
    checked(false),
    inSymmetry(false)
{}

// Constructor with all three coordinates.
Voxel::Voxel(int x, int y, int z) :
    x(x),
    y(y),
    z(z),
    superInteresting(false),
    interesting(false),
    checked(false),
    inSymmetry(false)
{}

// Constructor with all three coordinates and booleans for a super-interesting and an interesting voxel.
Voxel::Voxel(int x, int y, int z, bool si, bool i) :
    x(x),
    y(y),
    z(z),
    superInteresting(si),
    interesting(i),
    checked(false),
    inSymmetry(false)
{}

// Constructor with all three coordinates and a boolean for in-symmetry.
Voxel::Voxel(int x, int y, int z, bool is) :
    x(x),
    y(y),
    z(z),
    superInteresting(false),
    interesting(false),
    checked(false),
    inSymmetry(is)
{}

// Copy constructor with Point<> and a boolean for an interesting voxel.
Voxel::Voxel(Point<int>& p, bool i) :
    x(p.getX()),
    y(p.getY()),
    z(p.getZ()),
    superInteresting(i),
    interesting(i),
    checked(false),
    inSymmetry(false)
{}

// Copy constructor with Point<float>.
Voxel::Voxel(Point<float>& p) :
    x((int)p.getX()),
    y((int)p.getY()),
    z((int)p.getZ()),
    superInteresting(false),
    interesting(false),
    checked(false),
    inSymmetry(false)
{}



// GETTERS AND SETTERS
// Getting X coordinate.
int Voxel::getX() const {
    return x;
}

// Setting X coordinate.
void Voxel::setX(const int x) {
    this->x = x;
}

// Getting Y coordinate.
int Voxel::getY() const {
    return y;
}

// Setting Y coordinate.
void Voxel::setY(const int y) {
    this->y = y;
}

// Getting Z coordinate.
int Voxel::getZ() const {
    return z;
}

// Setting Z coordinate.
void Voxel::setZ(const int z) {
    this->z = z;
}

// Getting super-interesting property.
bool Voxel::getSuperInteresting() const {
    return superInteresting;
}

// Setting super-interesting property.
void Voxel::setSuperInteresting(const bool interesting) {
    this->superInteresting = interesting;
}

// Getting interesting property.
bool Voxel::getInteresting() const {
    return interesting;
}

// Setting interesting property.
void Voxel::setInteresting(const bool interesting) {
    this->interesting = interesting;
}

// Getting checked property.
bool Voxel::getChecked() const {
    return checked;
}

// Setting checked property.
void Voxel::setChecked(const bool checked) {
    this->checked = checked;
}

// Getting in symmetry property.
bool Voxel::getInSymmetry() const {
    return inSymmetry;
}

// Setting in symmetry property.
void Voxel::setInSymmetry(const bool inSymmetry) {
    this->inSymmetry = inSymmetry;
}



// CLASS METHODS
// Calculation of the Euclidian distance between two voxels.
double Voxel::distance(Voxel v1, Voxel v2) {
    return sqrt(
        pow(v1.x - v2.x, 2) +
        pow(v1.y - v2.y, 2) +
        pow(v1.z - v2.z, 2)
    );
}

// Getting the data whether the voxel should be painted brightly.
bool Voxel::isBrightNeighbor(const int x, const int y, const int z) {
    // First bright option.
    if (z % 2 == 0 && y % 2 == 0 && x % 2 == 0) {
        return true;
    }

    // Second bright option.
    if (z % 2 == 0 && y % 2 == 1 && x % 2 == 1) {
        return true;
    }

    // Third bright option.
    if (z % 2 == 1 && y % 2 == 1 && x % 2 == 0) {
        return true;
    }

    // Fourth bright option.
    if (z % 2 == 1 && y % 2 == 0 && x % 2 == 1) {
        return true;
    }

    return false;
}

// Getting the layer of the voxel mesh according to the Z coordinate.
int Voxel::getLayerInVoxelMesh(const int z, const VoxelMesh& vm) {
    int layer = (int)((z - vm.minZ) / vm.voxelSideSize);
    return layer;
}

// Returning interesting points from voxel vector.
std::vector<Point<float>> Voxel::getInterestingPointsFromVoxels(
    std::vector<std::vector<std::vector<Voxel>>> voxels,
    const float voxelEdge,
    const bool mustBeSuperInteresting
)
{
    std::vector<Point<float>> points;

    // Creating a new point from a voxel.
    for (unsigned long long i = 0; i < voxels.size(); i++) {
        for (unsigned long long j = 0; j < voxels[i].size(); j++) {
            for (unsigned long long k = 0; k < voxels[i][j].size(); k++) {
                if (!mustBeSuperInteresting || voxels[i][j][k].getSuperInteresting()) {
                    points.push_back(
                        Point<float>(
                            voxels[i][j][k].getX() + voxelEdge / 2,
                            voxels[i][j][k].getY() + voxelEdge / 2,
                            voxels[i][j][k].getZ() + voxelEdge / 2
                        )
                    );
                }
            }
        }
    }

    return points;
}



// OBJECT METHODS
// Checking whether the two voxels have the same coordinates.
bool Voxel::operator == (const Voxel v) const {
    if (x == v.x && y == v.y && z == v.z) {
        return true;
    }

    return false;
}

// Operator < serves for the sort method as the comparator.
bool Voxel::operator < (Voxel v) const {
    return (
        x < v.x ||
        (x == v.x && y < v.y) ||
        (x == v.x && y == v.y && z < v.z)
    );
}
