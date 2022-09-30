#include <pch.h>
#include "voxelgrid.h"

namespace Symmetry
{ 
    VoxelGrid::VoxelGrid()
    {

    }

    void VoxelGrid::generateVoxelGrid(std::vector<Point3D> points, double voxel_size)
    {
        double minX, minY, minZ, maxX, maxY, maxZ;
        calc_PC_BB(points, minX, minY, minZ, maxX, maxY, maxZ);

        int i, j, k;
        if(vox != nullptr)
        {
            for(i = 0; i < voxSX; i++)
            {
                for(j = 0; j < voxSY; j++)
                    delete[] vox[i][j];
                delete[] vox[i];
            }
            delete[] vox;
            vox = nullptr;
        }

        voxSX = (int)((maxX - minX) / voxel_size) + 1;
        voxSY = (int)((maxY - minY) / voxel_size) + 1;
        voxSZ = (int)((maxZ - minZ) / voxel_size) + 1;

        vox = new int** [voxSX];
        for(i = 0; i < voxSX; i++)
        {
            vox[i] = new int* [voxSY];
            for(j = 0; j < voxSY; j++)
            {
                vox[i][j] = new int[voxSZ];
                for(k = 0; k < voxSZ; k++)
                    vox[i][j][k] = 0;
            }
        }

        for(i = 0; i < points.size(); i++)
        {
            double x = (points[i].x - minX) / voxel_size;
            double y = (points[i].y - minY) / voxel_size;
            double z = (points[i].z - minZ) / voxel_size;

            vox[(int)x][(int)y][(int)z] = 1;
        }

        voxelGrid.clear();

        for(i = 0; i < voxSX; i++)
            for(j = 0; j < voxSY; j++)
                for(k = 0; k < voxSZ; k++)
                    if(vox[i][j][k] == 1)
                    {
                        Point3D v(i, j, k);
                        voxelGrid.push_back(v);
                    }

        moveToVoxCenter();
    }

    double VoxelGrid::getMiY() const
    {
        return miY;
    }

    double VoxelGrid::getMaY() const
    {
        return maY;
    }

    Point3D* VoxelGrid::getVoxelGridData()
    {
        return voxelGrid.data();
    }

    std::vector<Point3D>* VoxelGrid::getVoxelGrid()
    {
        return &voxelGrid;
    }

    void VoxelGrid::calc_PC_BB(std::vector<Point3D> points, double& minX, double& minY, double& minZ, double& maxX, double& maxY, double& maxZ)
    {
        int i;
        minX = points[0].x;
        minY = points[0].y;
        minZ = points[0].z;
        maxX = points[0].x;
        maxY = points[0].y;
        maxZ = points[0].z;

        for(i = 1; i < points.size(); i++)
        {
            if(points[i].x < minX) minX = points[i].x;
            if(points[i].y < minY) minY = points[i].y;
            if(points[i].z < minZ) minZ = points[i].z;
            if(points[i].x > maxX) maxX = points[i].x;
            if(points[i].y > maxY) maxY = points[i].y;
            if(points[i].z > maxZ) maxZ = points[i].z;
        }
    }

    void VoxelGrid::moveToVoxCenter()
    {
        int n = 0, i;
        double px = 0, py = 0, pz = 0;

        for(i = 0; i < voxelGrid.size(); i++)
        {
            px += voxelGrid[i].x + 0.5;
            py += voxelGrid[i].y + 0.5;
            pz += voxelGrid[i].z + 0.5;
            n++;
        }

        px /= (double)n;
        py /= (double)n;
        pz /= (double)n;

        for(i = 0; i < voxelGrid.size(); i++)
        {
            voxelGrid[i].x -= px;
            voxelGrid[i].y -= py;
            voxelGrid[i].z -= pz;
            if(i == 0)
            {
                miX = voxelGrid[0].x;
                miY = voxelGrid[0].y;
                miZ = voxelGrid[0].z;
                maX = voxelGrid[0].x;
                maY = voxelGrid[0].y;
                maZ = voxelGrid[0].z;
            }
            else
            {
                if(voxelGrid[i].x < miX) miX = voxelGrid[i].x;
                if(voxelGrid[i].y < miY) miY = voxelGrid[i].y;
                if(voxelGrid[i].z < miZ) miZ = voxelGrid[i].z;
                if(voxelGrid[i].x > maX) maX = voxelGrid[i].x;
                if(voxelGrid[i].y > maY) maY = voxelGrid[i].y;
                if(voxelGrid[i].z > maZ) maZ = voxelGrid[i].z;
            }
        }
    }

}//namespace Symmetry