#ifndef ANGLERANGEEVENT_H
#define ANGLERANGEEVENT_H

#include "voxel.h"


namespace Symmetry {
    // Class for angle ranges (while searching rotational symmetries).
    class AngleRangeEvent {
    private:
        // CLASS VARIABLES
        double angle;  // Angle in radians.
        int level;     // Level the event is part of (see documentation for more details).
        Voxel v;      // Voxel of the angle event.
        bool start;    // True if this is a starting event, false otherwise.

    public:
        // CONSTRUCTORS
        AngleRangeEvent();                                               // Default constructor without arguments.
        AngleRangeEvent(double angle, int level, Voxel v, bool start);  // Constructor with all necessary arguments.

        // GETTERS AND SETTERS
        double getAngle() const;            // Angle getter.
        void setAngle(const double angle);  // Angle setter.
        int getLevel() const;               // Level getter.
        void setLevel(const int level);     // Level setter.
        Voxel getVoxel() const;             // Voxel getter.
        void setVoxel(Voxel& v);            // Voxel setter.
        bool getStart() const;              // Start getter.
        void setStart(const bool start);    // Start setter.
    };
};

#endif // ANGLERANGEEVENT_H
