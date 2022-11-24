#ifndef POINTREADER_H
#define POINTREADER_H

#include <vector>

#include "../local_symmetry/helper_classes/point.h"



namespace Symmetry {
    namespace PointReader {
        std::vector<Point<float>> readPointsFromLAS(std::string path);  // Reading points from a LAS file.
    };
};

#endif // POINTREADER_H
