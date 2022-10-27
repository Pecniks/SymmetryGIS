#include <pch.h>
#include <algorithm>
#include <limits>
#include <map>
//#include <QDebug>
//#include <QString>
#include <queue>

#include "helper_classes/anglerangeevent.h"
#include "helper_classes/voxel.h"
#include "rotationalsymmetry.h"

using namespace Symmetry;



// PRIVATE OBJECT METHODS
// Function that increments the radius of the circumference around the center point
// with the step of 1. In every step it selects all interesting voxels the circumference
// overlaps with and calls the function FindRotationalSymmetries.
// In the end a union of all interesting voxels from obtained symmetries with different
// radii is made. The function returns a list of rotational symmetries (2 through 16),
// where each rotational symmetry is present once at most.
std::vector<RotationalSymmetry> RotationalSymmetry::getRotationalSymmetriesAroundCenterPoint(
    std::vector<std::vector<Voxel>>& field,
    Point<double> centerPoint,
    const uint16_t maxRadius,
    int32_t pixelX,
    int32_t pixelY)
{
    std::vector<RotationalSymmetry> tempSymmetries, symmetries, outSymmetries;
    std::queue<Voxel *> qu;
    std::vector<Voxel> interestingVoxels;
    Voxel* tp,* tp1;
    std::map<Voxel, bool> voxelsInSymmetry;

    // Maximum radius is the distance from the voxel to the nearest scene edge.
    uint16_t radius;
    double minD = 0.0;
    double maxD = 0.0;

    // Radius incrementation by 1.
    for (radius = 1; radius <= maxRadius; radius++) {
        interestingVoxels.clear();

        // Adding first voxel to the queue (same Y coordinate, for radius
        // away from the center point according to X coordinate.
        tp =& field[int(centerPoint.getY())][int(centerPoint.getX()) + radius];
        if (tp->getSuperInteresting()) {
            interestingVoxels.push_back(*tp);
        }
        tp->setChecked(true);
        qu.push(tp);

        // Four neighboring voxels of each voxel in queue are checked
        // (if not checked before and if the circumference is overlapping with them).
        // If true, voxels are added to the queue and set checked. If also interesting,
        // they are pushed to interestingVoxels.
        while (!qu.empty()) {
            tp = qu.front();

            // Left voxel.
            if (tp->getX() > 0) {
                tp1 =& field[tp->getY()][tp->getX() - 1];
                if (!tp1->getChecked()) {
                    RotationalSymmetry::getDistanceRanges(centerPoint,* tp1, minD, maxD);
                    if (minD <= double(radius) && maxD >= double(radius)) {
                        qu.push(tp1);
                        tp1->setChecked(true);
                        if (tp1->getSuperInteresting()) {
                            interestingVoxels.push_back(*tp1);
                        }
                    }
                }
            }

            // Bottom voxel.
            if (tp->getY() > 0) {
                tp1 =& field[tp->getY() - 1][tp->getX()];
                if (!tp1->getChecked()) {
                    RotationalSymmetry::getDistanceRanges(centerPoint,* tp1, minD, maxD);
                    if (minD <= double(radius) && maxD >= double(radius)) {
                        qu.push(tp1);
                        tp1->setChecked(true);
                        if (tp1->getSuperInteresting()) {
                            interestingVoxels.push_back(*tp1);
                        }
                    }
                }
            }

            // Right voxel.
            if (tp->getX() < pixelX - 1) {
                tp1 =& field[tp->getY()][tp->getX() + 1];
                if (!tp1->getChecked()) {
                    RotationalSymmetry::getDistanceRanges(centerPoint,* tp1, minD, maxD);
                    if (minD <= double(radius) && maxD >= double(radius)) {
                        qu.push(tp1);
                        tp1->setChecked(true);
                        if (tp1->getSuperInteresting()) {
                            interestingVoxels.push_back(*tp1);
                        }
                    }
                }
            }

            // Upper voxel.
            if (tp->getY() < pixelY - 1) {
                tp1 =& field[tp->getY() + 1][tp->getX()];
                if (!tp1->getChecked()) {
                    RotationalSymmetry::getDistanceRanges(centerPoint,* tp1, minD, maxD);
                    if (minD <= double(radius) && maxD >= double(radius)) {
                        qu.push(tp1);
                        tp1->setChecked(true);
                        if (tp1->getSuperInteresting()) {
                            interestingVoxels.push_back(*tp1);
                        }
                    }
                }
            }

            qu.pop();
        }

        // Resetting all checked voxels to false.
        tp =& field[int(centerPoint.getY())][int(centerPoint.getX()) + radius];
        tp->setChecked(false);
        qu.push(tp);
        while (!qu.empty()) {
            tp = qu.front();
            qu.pop();

            // Left voxel.
            if (tp->getX() > 0) {
                tp1 =& field[tp->getY()][tp->getX() - 1];
                if (tp1->getChecked()) {
                    tp1->setChecked(false);
                    qu.push(tp1);
                }
            }

            // Bottom voxel.
            if (tp->getY() > 0) {
                tp1 =& field[tp->getY() - 1][tp->getX()];
                if (tp1->getChecked()) {
                    tp1->setChecked(false);
                    qu.push(tp1);
                }
            }

            // Right voxel.
            if (tp->getX() < pixelX - 1) {
                tp1 =& field[tp->getY()][tp->getX() + 1];
                if (tp1->getChecked()) {
                    tp1->setChecked(false);
                    qu.push(tp1);
                }
            }

            // Upper voxel.
            if (tp->getY() < pixelY - 1) {
                tp1 =& field[tp->getY() + 1][tp->getX()];
                if (tp1->getChecked()) {
                    tp1->setChecked(false);
                    qu.push(tp1);
                }
            }
        }

        // Test of rotation symmetries in the set of interesting
        // voxels on current radius.
        if (interestingVoxels.size() > 1) {
            tempSymmetries = getRotationalSymmetriesOnRadius(
                centerPoint,
                interestingVoxels
            );


            // Appending found symmetries to the list of all symmetries.
            for (uint16_t i = 0; i < tempSymmetries.size(); i++) {
                symmetries.push_back(tempSymmetries[i]);
            }
        }
    }

    // Sorting of symmetries by the rotation level.
    std::sort(
        symmetries.begin(),
        symmetries.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs.getRotation() < rhs.getRotation();
        }
    );

    int16_t currentRotation = 0;
    int16_t cosi = -1;

    // Walking through the list of all symmetries and generating a symmetry for each rotation
    // (voxel union in symmetries, as some voxels may be present also in neighbouring symmetries
    // with the same rotation (two circumferences can overlap with the same voxel)).
    for (uint16_t i = 0; i < symmetries.size(); i++) {
        if (symmetries[i].getRotation() > currentRotation) {
            // If the rotation of the current symmetry is bigger that from the previous one,
            // a new symmetry with the given rotation is created.
            voxelsInSymmetry.clear();
            currentRotation = symmetries[i].getRotation();
            outSymmetries.push_back(symmetries[i]);
            cosi++;
            for (uint16_t j = 0; j < symmetries[i].getVoxelCount(); j++) {
                voxelsInSymmetry.insert(
                    std::pair<Voxel, bool>(symmetries[i].getVoxel(j), true)
                );
            }
        }
        else {
            // Merging symmetry with the previous one.
            for (uint16_t j = 0; j < symmetries[i].getVoxelCount(); j++) {
                // If there is no voxel in the symmetry, it is added to it.
                if (voxelsInSymmetry.count(symmetries[i].getVoxel(j)) == 0) {
                    voxelsInSymmetry.insert(
                        std::pair<Voxel, bool>(symmetries[i].getVoxel(j), true)
                    );
                    outSymmetries[cosi].addVoxel(symmetries[i].getVoxel(j));
                }
            }
        }
    }

    return outSymmetries;
}

// Merging symmetries by layers. Symmetries with the same search axis and rotation level are merged.
std::vector<RotationalSymmetry> RotationalSymmetry::mergeSymmetries(std::vector<RotationalSymmetry>& symmetries) {
    std::vector<RotationalSymmetry> mergedSymmetries;

    // Merging the found symmetries by Z axis and center point.
    for (RotationalSymmetry& symmetry : symmetries) {
        // Searching for a potential symmetry in the list, where X and
        // Y coordinates are the same as in the current symmetry, the
        // symmetry level is also the same.
        auto itr = std::find_if(
            mergedSymmetries.begin(),
            mergedSymmetries.end(),
            [&symmetry](const RotationalSymmetry curSymmetry) {
                Point<double> axis1 = symmetry.getSymmetryAxis();
                Point<double> axis2 = curSymmetry.getSymmetryAxis();
                return (
                    axis1.getX() == axis2.getX()
                    && axis1.getY() == axis2.getY()
                    && symmetry.getRotation() == curSymmetry.getRotation());
            }
        );

        // Adding the symmetry to the list if not exists.
        if (itr == mergedSymmetries.end()) {
            mergedSymmetries.push_back(symmetry);
        }
        // Updating the Symmetry object with new limits.
        else {
            auto index = std::distance(mergedSymmetries.begin(), itr);
            mergedSymmetries[index] &= symmetry;
        }
    }

    // Sorting the symmetries according to the rotation level.
    std::sort(
        mergedSymmetries.begin(),
        mergedSymmetries.end(),
        [](const RotationalSymmetry& s1, const RotationalSymmetry& s2) {
            return s1.getRotation() > s2.getRotation();
        }
    );

    return mergedSymmetries;
}

// A main function for searching symmetries gets a center point and a list of interesting voxels.
// Symmetries are searched for only in the set of interesting voxels that are on the same radius
// from the center point (or inside the same interval).
std::vector<RotationalSymmetry> RotationalSymmetry::getRotationalSymmetriesOnRadius(
    Point<double> centerPoint,
    const std::vector<Voxel>& interestingVoxels)
{
    uint16_t maxSymmetrySize = 16;               // Maximum symmetry level of the symmetry.
    std::vector<AngleRangeEvent> angleEvents;    // Vector of angle events.
    std::vector<uint16_t> activeLevels;          // Active levels.
    uint16_t activeLevelsCount;                  // Number of active levels.
    std::map<Voxel, uint16_t> activeVoxels;    // Map of active voxels.
    std::map<Voxel, uint16_t>::iterator pitr;  // Iterator through map.
    std::vector<bool> voxelsInSymmetry;          // Vector of voxels in symmetry.
    uint16_t voxelsInSymmetryCount = 0;          // Number of voxels in symmetry.
    std::vector<RotationalSymmetry> symmetries;  // List of different rotational symmetries.
    RotationalSymmetry tempSymmetry;             // Current symmetry.

    // Filling the vector of active levels with zeros.
    for (uint16_t i = 0; i < maxSymmetrySize; i++) {
        activeLevels.push_back(0);
    }

    // Filling the map with the pairs of interesting voxels and zeros.
    // Currently there have been no found voxels in the symmetry.
    for (uint16_t i = 0; i < interestingVoxels.size(); i++) {
        activeVoxels.insert(std::pair<Voxel, uint16_t>(interestingVoxels[i], 0));
        voxelsInSymmetry.push_back(false);
    }

    std::vector<RotationalSymmetry> tempSymmetries;                // Vector of currently found symmetries.
    std::vector<AngleRangeEvent> originalAngleEvents;    // Vector of current angle events.
    float minAngle = std::numeric_limits<float>::max();  // Minimum angle.
    float maxAngle = std::numeric_limits<float>::min();  // Maximum angle.

    // Appending angle events for all interesting voxels.
    for (uint32_t i = 0; i < interestingVoxels.size(); i++) {
        // Minimum and maximum angle calculation.
        RotationalSymmetry::getAngleRanges(centerPoint, interestingVoxels[i], minAngle, maxAngle);

        // If the angle is >PI, the interval is split into 2 parts.
        if (maxAngle - minAngle > M_PI) {
            originalAngleEvents.push_back(
                AngleRangeEvent(0, 0, interestingVoxels[i], true)
            );
            originalAngleEvents.push_back(
                AngleRangeEvent(minAngle, 0, interestingVoxels[i], false)
            );
            originalAngleEvents.push_back(
                AngleRangeEvent(maxAngle, 0, interestingVoxels[i], true)
            );
            originalAngleEvents.push_back(
                AngleRangeEvent(M_PI * 2, 0, interestingVoxels[i], false)
            );
        }
        else {
            originalAngleEvents.push_back(
                AngleRangeEvent(minAngle, 0, interestingVoxels[i], true)
            );
            originalAngleEvents.push_back(
                AngleRangeEvent(maxAngle, 0, interestingVoxels[i], false)
            );
        }
    }

    // Sorting events by angle.
    std::sort(
        originalAngleEvents.begin(),
        originalAngleEvents.end(),
        [](auto& lhs, auto& rhs) {
            return lhs.getAngle() < rhs.getAngle();
        }
    );

    std::map<Voxel, bool> activeInterval;  // An active interval.
    std::map<Voxel, bool>::iterator itr;   // Iterator through intervals.

    // Appending the active intervals for each interesting voxel.
    for (uint32_t i = 0; i < interestingVoxels.size(); i++) {
        activeInterval.insert({std::make_pair(interestingVoxels[i], false)});
    }

    uint16_t level = 0;     // Symmetry level.
    uint16_t newLevel = 0;  // New symmetry level.
    double rotationAngle;   // Angle of the rotation.

    // Marching through all the symmetries.
    for (uint32_t i = 2; i <= maxSymmetrySize; i++) {
        // Cleaning variables from the previous loop iteration.
        for (itr = activeInterval.begin(); itr != activeInterval.end(); ++itr) {
            itr->second = false;
        }
        angleEvents.clear();
        level = 0;
        rotationAngle = 2 * M_PI / i;  // Calculation of the rotation symmetry angle.

        // Creating intervals on all levels.
        for (uint16_t j = 0; j < originalAngleEvents.size(); j++) {
            newLevel = uint16_t(std::floor(originalAngleEvents[j].getAngle() / rotationAngle));
            if (newLevel > 0
                && !originalAngleEvents[j].getStart()
                && originalAngleEvents[j].getAngle() / rotationAngle - double(newLevel) == 0.0)
            {
                newLevel--;
            }

            if (newLevel > level) {
                // Initialization of the new levels start.
                level = newLevel;
                for (itr = activeInterval.begin(); itr != activeInterval.end(); ++itr) {
                    if (itr->second) {
                        angleEvents.push_back(AngleRangeEvent(0, level, itr->first, true));
                    }
                }
            }

            // Adding new angle event.
            angleEvents.push_back(
                AngleRangeEvent(
                    originalAngleEvents[j].getAngle() - (newLevel * rotationAngle),
                    newLevel,
                    originalAngleEvents[j].getVoxel(),
                    originalAngleEvents[j].getStart()
                )
            );
            activeInterval[originalAngleEvents[j].getVoxel()] = originalAngleEvents[j].getStart();
        }

        // Sorting all angle events by angle and start.
        std::sort(angleEvents.begin(), angleEvents.end(), [](auto& lhs, auto& rhs) {
            return lhs.getStart() < rhs.getStart();
        });
        std::sort(angleEvents.begin(), angleEvents.end(), [](auto& lhs, auto& rhs) {
            return lhs.getAngle() < rhs.getAngle();
        });

        // Checking voxels that are present in intervals.
        for (uint16_t j = 0; j < maxSymmetrySize; j++) {
            activeLevels[j] = 0;
        }
        for (uint16_t j = 0; j < interestingVoxels.size(); j++) {
            voxelsInSymmetry[j] = false;
        }
        for (pitr = activeVoxels.begin(); pitr != activeVoxels.end(); pitr++) {
            pitr->second = 0;
        }
        activeLevelsCount = 0;
        voxelsInSymmetryCount = 0;

        for (uint16_t j = 0; j < angleEvents.size(); j++) {
            if (angleEvents[j].getStart()) {
                activeLevels[angleEvents[j].getLevel()]++;
                activeVoxels[angleEvents[j].getVoxel()]++;
                if (activeLevels[angleEvents[j].getLevel()] == 1) {
                    activeLevelsCount++;
                }
                if (activeLevelsCount == i) {
                    for (unsigned long long k = 0; k < voxelsInSymmetry.size(); k++) {
                        if (voxelsInSymmetry[k] == false)
                            voxelsInSymmetryCount++;
                        voxelsInSymmetry[k] = true;
                    }
                }
            }
            else {
                activeLevels[angleEvents[j].getLevel()]--;
                activeVoxels[angleEvents[j].getVoxel()]--;
                if (activeLevels[angleEvents[j].getLevel()] == 0) {
                    activeLevelsCount--;
                }
            }
        }

        // If no symmetry, a next iteration of the loop follows.
        if (voxelsInSymmetryCount == 0) {
            continue;
        }

        // Creating the rotational symmetry and appending to the
        // list of all symmetries on current radius.
        const int layer = (int)centerPoint.getZ();
        tempSymmetry = RotationalSymmetry(centerPoint, i, layer, layer);

        // Adding voxels to the current symmetry.
        for (uint16_t j = 0; j < voxelsInSymmetry.size(); j++) {
            if (voxelsInSymmetry[j]) {
                tempSymmetry.addVoxel(interestingVoxels[j]);
            }
        }

        // Adding symmetry to the list of all symmetries.
        symmetries.push_back(tempSymmetry);
    }

    return symmetries;
}



// CONSTRUCTOR AND DESTRUCTOR
// Default constructor.
RotationalSymmetry::RotationalSymmetry() :
    symmetryAxis(Point<double>()),
    rotation(0),
    upperLayer(0),
    lowerLayer(0)
{}

// Constructor with all parameters.
RotationalSymmetry::RotationalSymmetry(
    const Point<double>& symmetryAxis,
    const uint8_t& rotation,
    const uint32_t& upperLayer,
    const uint32_t& lowerLayer
) :
    symmetryAxis(symmetryAxis),
    rotation(rotation),
    upperLayer(upperLayer),
    lowerLayer(lowerLayer)
{}

// Destructor.
RotationalSymmetry::~RotationalSymmetry() {
//    for (unsigned int i = 0; i < this->points.size(); i++) {
//        delete this->points[i];
//    }
}



// OVERLOADED OPERATORS
// Operator & is used for merging two symmetries.
RotationalSymmetry RotationalSymmetry::operator & (const RotationalSymmetry& s) const {
    RotationalSymmetry newS = RotationalSymmetry(*this);   // New symmetry.

    // Searching for minimum and maximum layer inside the both symmetries.
    int32_t minLayer = std::min(this->getLowerLayer(), s.getLowerLayer());
    int32_t maxLayer = std::max(this->getUpperLayer(), s.getUpperLayer());

    // Setting new upper and lower layer.
    newS.setLowerLayer(minLayer);
    newS.setUpperLayer(maxLayer);

    return newS;
}

// Operator &= is used for merging the current symmetry with another.
RotationalSymmetry RotationalSymmetry::operator &= (RotationalSymmetry& s) {
    // Searching for minimum and maximum layer inside the both symmetries.
    int32_t minLayer = std::min(this->getLowerLayer(), s.getLowerLayer());
    int32_t maxLayer = std::max(this->getUpperLayer(), s.getUpperLayer());

    // Setting new upper and lower layer.
    this->lowerLayer = minLayer;
    this->upperLayer = maxLayer;

    // Adding the voxels of the second symmetry.
    std::vector<Voxel> voxels = s.getVoxels();
    for (unsigned int i = 0; i < voxels.size(); i++) {
        this->addVoxel(voxels[i]);
    }

    return* this;
}



// GETTERS AND SETTERS
// Getting voxel count.
int RotationalSymmetry::getVoxelCount() const {
    return (int)voxels.size();
}

// Getting all voxels.
std::vector<Voxel>& RotationalSymmetry::getVoxels() {
    return voxels;
}

// Getting a voxel at a certain index.
Voxel RotationalSymmetry::getVoxel(int index) const {
    return voxels[index];
}

// Adding a voxel to the vector.
void RotationalSymmetry::addVoxel(Voxel v) {
    voxels.push_back(v);
}

// Getting a symmetry axis.
Point<double> RotationalSymmetry::getSymmetryAxis() const {
    return symmetryAxis;
}

// Setting a symmetry axis.
void RotationalSymmetry::setSymmetryAxis(Point<double> symmetryAxis) {
    this->symmetryAxis = symmetryAxis;
}

// Getting a rotation.
uint8_t RotationalSymmetry::getRotation() const {
    return rotation;
}

// Setting a rotation.
void RotationalSymmetry::setRotation(uint8_t rotation) {
    this->rotation = rotation;
}

// Getting an upper layer.
uint32_t RotationalSymmetry::getUpperLayer() const {
    return upperLayer;
}

// Setting an upper layer.
void RotationalSymmetry::setUpperLayer(uint32_t upperLayer) {
    this->upperLayer = upperLayer;
}

// Getting a lower layer.
uint32_t RotationalSymmetry::getLowerLayer() const {
    return lowerLayer;
}

// Setting a lower layer.
void RotationalSymmetry::setLowerLayer(uint32_t lowerLayer) {
    this->lowerLayer = lowerLayer;
}



// CLASS METHODS
// Symmetry search class initialization method.
void RotationalSymmetry::setGeometrySearch(int32_t x, int32_t y) {
    // Creating 2D array for vicinity scan.
    for (int32_t i = 0; i < x; i++) {
        RotationalSymmetry::geometrySearch.push_back(std::vector<bool>());

        for (int32_t j = 0; j < y; j++) {
            RotationalSymmetry::geometrySearch[i].push_back(false);
        }
    }
    RotationalSymmetry::geometrySearchSizeX = x;  // Size of scan by X.
    RotationalSymmetry::geometrySearchSizeY = y;  // Size of scan by Y.
}

// Cleaning the history of geometry search.
void RotationalSymmetry::clearGeometrySearch(int32_t x, int32_t y) {
    std::queue<int32_t> coordsQueue;  // Queue for the search.
    geometrySearch[x][y] = false;
    coordsQueue.push(x);
    coordsQueue.push(y);

    // Scanning the vicinity.
    while (!coordsQueue.empty()) {
        x = coordsQueue.front();
        coordsQueue.pop();

        y = coordsQueue.front();
        coordsQueue.pop();

        // Left point.
        if (x > 0) {
            if (geometrySearch[x - 1][y]) {
                geometrySearch[x - 1][y] = false;
                coordsQueue.push(x - 1);
                coordsQueue.push(y);
            }
        }
        // Right point.
        if (x < geometrySearchSizeX - 1) {
            if (geometrySearch[x + 1][y]) {
                geometrySearch[x + 1][y] = false;
                coordsQueue.push(x + 1);
                coordsQueue.push(y);
            }
        }
        // Bottom point.
        if (y > 0) {
            if (geometrySearch[x][y - 1]) {
                geometrySearch[x][y - 1] = false;
                coordsQueue.push(x);
                coordsQueue.push(y - 1);
            }
        }
        // Top point.
        if (y < geometrySearchSizeY - 1) {
            if (geometrySearch[x][y + 1]) {
                geometrySearch[x][y + 1] = false;
                coordsQueue.push(x);
                coordsQueue.push(y + 1);
            }
        }
    }
}

// Calculation of angles of rotational symmetries (interval [1, 16]).
void RotationalSymmetry::setSymmetryAngles() {
    symmetryAngles.push_back(FP_INFINITE);

    // Angle calculation of a rotational symmetry.
    for (uint8_t i = 1; i < 16; i++) {
        symmetryAngles.push_back((float)(2 * M_PI / i));
    }
}

// Calculation of the minimum and the maximum distance from the center point to the voxel (in 2D).
void RotationalSymmetry::getDistanceRanges(Point<double> centerPoint, Voxel v, double& minD, double& maxD) {
    minD = 1000000.0;
    maxD = 0.0;
    Point<double> dp(v.getX(), v.getY(), v.getZ());

    double D = Point<double>::distance(centerPoint, dp);
    if (D < minD) {
        minD = D;
    }
    if (D > maxD) {
        maxD = D;
    }
    dp.setX(dp.getX() + 1.0);

    D = Point<double>::distance(centerPoint, dp);
    if (D < minD) {
        minD = D;
    }
    if (D > maxD) {
        maxD = D;
    }
    dp.setY(dp.getY() + 1.0);

    D = Point<double>::distance(centerPoint, dp);
    if (D < minD) {
        minD = D;
    }
    if (D > maxD) {
        maxD = D;
    }
    dp.setX(dp.getX() - 1.0);

    D = Point<double>::distance(centerPoint, dp);
    if (D < minD) {
        minD = D;
    }
    if (D > maxD) {
        maxD = D;
    }
}

// Getting the angle between two vectors (represented as Points).
float RotationalSymmetry::getAngle(const Point<double> a, const Point<double> b) {
    float dot = (float)(a.getX() * b.getX()) + (float)(a.getY() * b.getY());
    float det = (float)(a.getX() * b.getY()) - (float)(a.getY() * b.getX());
    float angle = std::atan2(det, dot);

    // Angle lies inside the interval [0, 2 * PI].
    return (float)(angle >= 0 ? angle : angle + M_PI * 2);
}

// Calculation of the minimum and the maximum angle between horizontal vector
// and the vectors from the center point to the vertices of the voxel (in 2D).
void RotationalSymmetry::getAngleRanges(
    const Point<double>& centerPoint,
    Voxel v,
    float& minAngle,
    float& maxAngle)
{
    minAngle = (float)(2 * M_PI);
    maxAngle = 0.0;
    Point<double> vertical(1.0, 0.0, 0.0);
    Point<double> dp(v.getX(), v.getY(), v.getZ());


    if (double(v.getX()) > centerPoint.getX()
        && centerPoint.getY() - double(v.getY()) < 1.0
        && centerPoint.getY() - double(v.getY()) > 0.0)
    {
        maxAngle = getAngle(vertical, dp - centerPoint);
        dp.setY(dp.getY() + 1);
        minAngle = getAngle(vertical, dp - centerPoint);
        return;
    }

    float angle = getAngle(vertical, dp - centerPoint);
    if (angle < minAngle)
        minAngle = angle;
    if (angle > maxAngle)
        maxAngle = angle;
    dp.setX(dp.getX() + 1.0);

    angle = getAngle(vertical, dp - centerPoint);
    if (angle < minAngle)
        minAngle = angle;
    if (angle > maxAngle)
        maxAngle = angle;
    dp.setY(dp.getY() + 1.0);

    angle = getAngle(vertical, dp - centerPoint);
    if (angle < minAngle)
        minAngle = angle;
    if (angle > maxAngle)
        maxAngle = angle;
    dp.setX(dp.getX() - 1.0);

    angle = getAngle(vertical, dp - centerPoint);
    if (angle < minAngle)
        minAngle = angle;
    if (angle > maxAngle)
        maxAngle = angle;
}



// OBJECT METHODS
// Finding rotational symmetries.
std::vector<RotationalSymmetry> RotationalSymmetry::getRotationalSymmetries(
    std::vector<std::vector<std::vector<Voxel>>> voxels,
    VoxelMesh& voxelMesh)
{
    std::vector<RotationalSymmetry> symmetries;      // Vector of all symmetries.
    uint16_t maxRadius;                              // Maximum radius for searching rotational symmetries.
    int step = 10 * 1;                               // Step in voxel mesh.
    std::vector<RotationalSymmetry> tempSymmetries;  // Vector of symmetries on the radius.

    // Marching the voxel mesh of all layers and finding symmetries.
    for (int layer = (int)voxels.size() - 1; layer >= 0; layer--) {
        for (int i = 15; i < 10 * (voxelMesh.voxelX - 1); i += step) {
            for (int j = 15; j < 10 * (voxelMesh.voxelY - 1); j += step) {
                float x = i / 10.0f;
                float y = j / 10.0f;

                // Setting the coordinates of a new center
                // point, where rotational symmetries are searched.
                Point<double> centerPoint(x, y, layer);

                // Maximum radius where symmetries are searched for, equals
                // the distance to the nearest edge of the bounding box.
                maxRadius = (int)std::min(
                    std::min(voxelMesh.voxelX - x, voxelMesh.voxelY - y),
                    std::min(x, y)
                );

                // Searching rotational symmetries around the current point.
                tempSymmetries = getRotationalSymmetriesAroundCenterPoint(
                    voxels[layer],
                    centerPoint,
                    maxRadius,
                    voxelMesh.voxelX,
                    voxelMesh.voxelY
                );

                // Inserting the found symmetries into the vector of all symmetries.
                symmetries.insert(
                    std::end(symmetries),
                    std::begin(tempSymmetries),
                    std::end(tempSymmetries)
                );
            }
        }
    }

    // Merging symmetries according to the axis and the rotation.
    std::vector<RotationalSymmetry> mergedSymmetries = mergeSymmetries(symmetries);

    return mergedSymmetries;
}

// Calculating the point positions according to the axis.
std::vector<PositionFromPlane> RotationalSymmetry::calculatePositionsFromAxis(const Point<double>* axis) {
    const unsigned long long numberOfPoints = points.size();  // Getting the point vector size.

    // If there is no plane, all points are undefined.
    if (axis == nullptr) {
        return std::vector<PositionFromPlane>(numberOfPoints, PositionFromPlane::undefined);
    }

    // Creating a new vector with positions.
    std::vector<PositionFromPlane> positions(numberOfPoints, PositionFromPlane::undefined);
    for (unsigned long long i = 0; i < numberOfPoints; i++) {
        // Getting each single point and its voxel coordinates.
        const Point<float>& point = points[i];
        const int voxelX = (int)floor((point.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
        const int voxelY = (int)floor((point.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
        const int voxelZ = (int)floor((point.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);

        // If the current point lies outside of an interesting voxel, its position is set to undefined.
        if (std::find(voxels.begin(), voxels.end(), Voxel(voxelX, voxelY, voxelZ)) == voxels.end()) {
             positions[i] = PositionFromPlane::undefined;
             continue;
        }

        // Otherwise, the position is set to rotational.
        positions[i] = PositionFromPlane::rotational;
    }

    return positions;
}
