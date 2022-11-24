#ifndef SYMRESULT_H
#define SYMRESULT_H

#include <vector>

#include "helper_classes/linesegment.h"
#include "helper_classes/plane.h"


namespace Symmetry {
    template <typename T>
    class SymResult {
    public:
        T symScore;
    };

    template <typename T>
    class ReflectionSymmetryResult : public SymResult<T> {
    public:
        Plane symPlane;
        std::vector<LineSegment> lineSegments;
        std::vector<Voxel> voxels;
    };
};

#endif // SYMRESULT_H
