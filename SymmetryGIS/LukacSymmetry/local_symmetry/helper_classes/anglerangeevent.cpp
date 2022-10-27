#include <pch.h>
#include "anglerangeevent.h"

using namespace Symmetry;



// CONSTRUCTORS
// Default constructor without arguments.
AngleRangeEvent::AngleRangeEvent() :
    angle(0.0),
    level(0),
    start(false)
{}

// Constructor with all necessary arguments.
AngleRangeEvent::AngleRangeEvent(double angle, int level, Voxel v, bool start) :
    angle(angle),
    level(level),
    v(v),
    start(start)
{}



// GETTERS AND SETTERS
// Angle getter.
double AngleRangeEvent::getAngle() const {
    return angle;
}

// Angle setter.
void AngleRangeEvent::setAngle(const double angle) {
    this->angle = angle;
}

// Level getter.
int AngleRangeEvent::getLevel() const {
    return level;
}

// Level setter.
void AngleRangeEvent::setLevel(const int level) {
    this->level = level;
}

// Voxel getter.
Voxel AngleRangeEvent::getVoxel() const {
    return v;
}

// Voxel setter.
void AngleRangeEvent::setVoxel(Voxel& v) {
    this->v = v;
}

// Start getter.
bool AngleRangeEvent::getStart() const {
    return start;
}

// Start setter.
void AngleRangeEvent::setStart(bool start) {
    this->start = start;
}
