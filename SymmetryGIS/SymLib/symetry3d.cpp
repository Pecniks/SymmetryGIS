#include "symetry3d.h"
#include <math.h>
#include <omp.h>

Symetry3D::Symetry3D()
{
    int i;
    for(i=0;i<361;i++) {
        sinTable[i]=round_off(sin(i * PI / 180.0),5);
        cosTable[i]=round_off(cos(i * PI / 180.0),5);
    }

    omp_init_lock(&mutex);
}

Symetry3D::~Symetry3D() {
     omp_destroy_lock(&mutex);
}

double Symetry3D::round_off(double val, unsigned int prec) {
    double pow_10 = pow(10.0f, (double)prec);
    if(fabs(val) < 1E-7)
        val=0;
    return round(val * pow_10) / pow_10;
}

SymetryPlaneData Symetry3D::determineSymetryPlane(std::vector<Point3D> points, double vox_size, double coarse_factor)
{
    grid.generateVoxelGrid(points,vox_size);
    coarseGrid.generateVoxelGrid(points,vox_size * coarse_factor);

    double array[360];

    //evaluateSymetriesNClass(VoxelGrid *voxGrid, double array[92][362], int angleStep, double interval, double &minSym, double &maxSym)
    //sym3dCalc.evaluateSymetriesNClass(&coarseGrid,ang,ui->symPlaneAngleStep->value(),SPLIT3D_LIMIT,minSym,maxSym);
    double minSym=100000;
    double maxSym=-100000;

    int N=coarseGrid.getVoxelGrid()->size();

    #pragma omp parallel for
    for(int val=0;val<360;val++) {
        int theta=val;

        Vector3D symN;
        float symetry=evaluateSymetryNClass(true, N, symN, theta, SPLIT3D_LIMIT);

        omp_set_lock(&mutex);
        array[theta]=symetry;

        if(minSym > symetry) {
            minSym=symetry;
        }
        if(maxSym < symetry)
            maxSym=symetry;

        omp_unset_lock(&mutex);
    }

    //estimateBestSymetryNClass(VoxelGrid *voxGrid, double array[92][362], int angleStep, double interval, double minSym, double maxSym)
    //SymetryPlaneData sym=sym3dCalc.estimateBestSymetryNClass(&grid,ang,ui->symPlaneAngleStep->value(),SPLIT3D_LIMIT,minSym,maxSym);
    SymetryPlaneData symNormal;
    symNormal.symetry=100000;

    double bestRelSym=minSym / maxSym;
    N=grid.getVoxelGrid()->size();

    int counter=0;

    #pragma omp parallel for
    for(int val=0;val<360;val++) {
        int theta=val;
        double relSym=array[theta] / maxSym;
        if(relSym - bestRelSym < 0.05) {
            Vector3D symN;
            float symetry=evaluateSymetryNClass(false, N, symN, theta, SPLIT3D_LIMIT);

            omp_set_lock(&mutex);
            counter++;
            if(symetry < symNormal.symetry) {
                symNormal.symetry=symetry;
                symNormal.normal=symN;
                symNormal.angle=theta;
            }
            omp_unset_lock(&mutex);
        }
    }

    return symNormal;
}

double Symetry3D::evaluateSymetryNClass(bool coarse, int N, Vector3D &symN, int theta, double interval) {
    int M,i,index;

    Vector3D splitN;
    VoxelGrid *voxGrid = (coarse) ? &coarseGrid : &grid;
    double d, sumOcena;

    symN.x=sinTable[theta];
    symN.y=0;
    symN.z=cosTable[theta];

    splitN.x=0;
    splitN.y=1;
    splitN.z=0;

    double dMax, dMin;

    dMax=voxGrid->getMaY();
    dMin=voxGrid->getMiY();

    M=(int)((dMax - dMin) / interval) + 1;

    SymClass *classesL=new SymClass[M],*classesR=new SymClass[M];

    for(i=0;i<M;i++) {
        classesL[i].counter=0;
        classesR[i].counter=0;
    }

    Point3D *voxGrd=voxGrid->getVoxelGridData();
    Vector3D v,lRefl;

    double r=VOX_RADIUS*fabs(symN.x) + VOX_RADIUS*fabs(symN.y) + VOX_RADIUS*fabs(symN.z),s;

    for(i=0;i<N;i++) {
        v.x=voxGrd[i].x + 0.5;
        v.y=voxGrd[i].y + 0.5;
        v.z=voxGrd[i].z + 0.5;

        s=symN.x * v.x + symN.y * v.y + symN.z * v.z;

        if(fabs(s) <= r) {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / interval);

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
        }
        //left
        else if(s < 0) {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / interval);

            classesL[index].COG.x += v.x;
            classesL[index].COG.y += v.y;
            classesL[index].COG.z += v.z;
            classesL[index].counter++;
        }
        //right
        else {
            d=splitN.x * v.x + splitN.y * v.y + splitN.z * v.z;

            if(d>dMax)
                d=dMax;
            if(d<dMin)
                d=dMin;

            index=(int)((d-dMin) / interval);

            classesR[index].COG.x += v.x;
            classesR[index].COG.y += v.y;
            classesR[index].COG.z += v.z;
            classesR[index].counter++;
        }
    }

    for(i=0;i<M;i++) {
        if(classesL[i].counter > 0) {
            classesL[i].COG.x /= (double)classesL[i].counter;
            classesL[i].COG.y /= (double)classesL[i].counter;
            classesL[i].COG.z /= (double)classesL[i].counter;
        }
        if(classesR[i].counter > 0) {
            classesR[i].COG.x /= (double)classesR[i].counter;
            classesR[i].COG.y /= (double)classesR[i].counter;
            classesR[i].COG.z /= (double)classesR[i].counter;
        }
    }

    sumOcena=0;
    int myCounter=0;
    for(i=0;i<M;i++) {
        if(classesL[i].counter != 0 && classesR[i].counter != 0) {
            lRefl.x=(1-2*symN.x*symN.x)*classesL[i].COG.x    -2*symN.x*symN.y  *classesL[i].COG.y     -2*symN.x*symN.z *classesL[i].COG.z;
            lRefl.y=  -2*symN.x*symN.y *classesL[i].COG.x + (1-2*symN.y*symN.y)*classesL[i].COG.y     -2*symN.y*symN.z *classesL[i].COG.z;
            lRefl.z=  -2*symN.x*symN.z *classesL[i].COG.x     -2*symN.z*symN.y *classesL[i].COG.y + (1-2*symN.z*symN.z)*classesL[i].COG.z;
            sumOcena += lRefl.distanceToPoint(classesR[i].COG);
            myCounter++;
        }
        else if(classesL[i].counter != 0) {
            sumOcena += fabs(symN.x * classesL[i].COG.x + symN.y * classesL[i].COG.y + symN.z * classesL[i].COG.z);
            myCounter++;
        }
        else if(classesR[i].counter != 0) {
            sumOcena += fabs(symN.x * classesR[i].COG.x + symN.y * classesR[i].COG.y + symN.z * classesR[i].COG.z);
            myCounter++;
        }
    }

    delete[] classesL;
    delete[] classesR;

    return (sumOcena / (double)myCounter);
}
