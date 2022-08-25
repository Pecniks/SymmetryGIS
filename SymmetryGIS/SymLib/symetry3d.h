#ifndef SYMETRY3D_H
#define SYMETRY3D_H

#include "voxelgrid.h"
#include "struct_const.h"
#include <vector>
#include <math.h>
#include <omp.h>

class Symetry3D
{
public:
    Symetry3D();

    ~Symetry3D();

    SymetryPlaneData determineSymetryPlane(std::vector<Point3D> points, double vox_size=0.1, double coarse_factor=10);

private:
    VoxelGrid grid,coarseGrid;
    double sinTable[361],cosTable[361];

    double round_off(double val, unsigned int prec);
    double evaluateSymetryNClass(bool coarse, int N, Vector3D &symN, int theta, double interval);

    omp_lock_t mutex;
};

#endif // SYMETRY3D_H
