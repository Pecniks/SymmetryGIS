#ifndef TOLERANCE_H
#define TOLERANCE_H


namespace Symmetry {
    class Tolerance {
    public:
        // CLASS METHODS
        static bool isInTolerance(const double v1, const double v2, const double tolerance);  // Returning true if the two values differ for a value less than allowed tolerance.
    };
};

#endif // TOLERANCE_H
