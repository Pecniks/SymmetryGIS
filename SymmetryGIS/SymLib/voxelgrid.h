#ifndef VOXELGRID_H
#define VOXELGRID_H

#include "struct_const.h"
#include <vector>

namespace Symmetry
{
    class VoxelGrid
    {
    public:
        VoxelGrid();

        void generateVoxelGrid(std::vector<Point3D> points, double voxel_size = 0.1);

        double getMiY() const;

        double getMaY() const;

        Point3D* getVoxelGridData();

        std::vector<Point3D>* getVoxelGrid();

    private:
        int*** vox;
        int voxSX, voxSY, voxSZ;
        std::vector<Point3D> voxelGrid;
        double miX, miY, miZ, maX, maY, maZ;

        void calc_PC_BB(std::vector<Point3D> points, double& minX, double& minY, double& minZ, double& maxX, double& maxY, double& maxZ);
        void moveToVoxCenter();
    };
}// Symmetry


#endif // VOXELGRID_H
