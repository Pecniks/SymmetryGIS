#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "local_symmetry/bruterotationalsymmetry.h"
#include "local_symmetry/reflectionsymmetry.h"
#include "local_symmetry/rotationalsymmetry.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

using namespace Symmetry;


class MainWindow : public QMainWindow {
    Q_OBJECT
private:
    // OBJECT VARIABLES
    Ui::MainWindow* ui;                                                      // Qt GUI object.
    BruteRotationalSymmetry rotationalSymmetry;                                   // Object for rotational symmetry functions.
    std::vector<BruteRotationalSymmetry> rotationalSymmetries;                    // Vector of rotational symmetries.
    std::vector<RotationalSymmetry> rotationalSymmetriesWise;                    // Vector of rotational symmetries.
    std::vector<ReflectionSymmetryResult<int>> reflectionSymmetries;                    // Vector of reflection symmetries.
    std::vector<ReflectionSymmetry> partialReflectionSymmetries;             // Vector of partial reflection symmetries.

private slots:
    // SLOTS
    void on_actionAbout_triggered();                                         // Displaying the basic data about the app.
    void on_actionResetCamera_triggered();                                   // Resetting the camera position and rotation to initial values.
    void on_actionTop_down_view_triggered();                                 // Camera top-down view according to the bounding box.
    void on_actionCameraParameters_triggered();                              // Setting the camera parameters.
    void on_btn_ChooseFile_clicked();                                        // Function for choosing a LAS file.
    void on_btn_LoadPoints_clicked();                                        // Loading and rendering points.
    void on_rb_VoxelizationSideLength_clicked();                             // Selecting voxel calculation by side length.
    void on_rb_VoxelizationCount_clicked();                                  // Selecting voxel calculation by maximum count.
    void on_btn_CalculateVoxels_clicked();                                   // Voxel mesh calculation.
    void on_btn_FindInterestingVoxels_clicked();                             // Finding interesting voxels.
    void on_btn_FindReflectionSymmetries_clicked();                          // Reflection symmetry search procedure.
    void on_btn_FindRotationalSymmetriesBrute_clicked();                     // Rotational symmetries search procedure (wise).
    void on_btn_FindRotationalSymmetries_clicked();                          // Rotational symmetries search procedure.

    void on_rb_PerspectiveProjection_clicked();                              // Setting perspective OpenGL projection.
    void on_rb_OrthoProjection_clicked();                                    // Setting orthogonal OpenGL projection.
    void on_cbx_RenderBoundingBox_stateChanged(int arg1);                    // Enabling/disabling bounding box rendering in OpenGL.
    void on_cbx_RenderPoints_stateChanged(int arg1);                         // Enabling/disabling point rendering.
    void on_rb_RenderVoxelEdges_clicked();                                   // Rendering voxels as edges.
    void on_rb_RenderVoxelCubes_clicked();                                   // Rendering voxels as cubes.
    void on_cbx_RenderVoxels_stateChanged(int arg1);                         // Enabling/disabling voxel rendering in OpenGL.
    void on_cbx_RenderOrdinaryVoxels_stateChanged(int arg1);                 // Enabling/disabling ordinary voxels rendering in OpenGL.
    void on_cbx_RenderSuperInterestingVoxels_stateChanged(int arg1);         // Enabling/disabling super-interesting voxels rendering in OpenGL.
    void on_cbx_RenderInterestingVoxels_stateChanged(int arg1);              // Enabling/disabling interesting voxels rendering in OpenGL.
    void on_cbx_RenderSymmetryVoxels_stateChanged(int arg1);                 // Enabling/disabling symmetry voxels rendering in OpenGL
    void on_cbx_RenderSymmetryPlane_stateChanged(int arg1);                  // Enabling/disabling symmetry plane rendering.
    void on_cbx_RenderLineSegments_stateChanged(int arg1);                   // Enabling/disabling line segment rendering.
    void on_rb_VisualizationLocalReflectionSymmetries_clicked();             // Local reflection symmetries list display.
    void on_rb_VisualizationPartialReflectionSymmetries_clicked();           // Partial reflection symmetries list display.
    void on_cbx_ReflectionSymmetries_currentIndexChanged(int index);         // Displaying a chosen reflection symmetry.
    void on_cbx_ReflectionSymmetriesPartial_currentIndexChanged(int index);  // Displaying a chosen partial reflection symmetry.
    void on_btn_ReflectionSymmetryTopView_clicked();                         // Moving the camera to the top of the reflection symmetry.
    void on_cbx_RenderSymmetryAxis_stateChanged(int arg1);                   // Enabling/disabling symmetry axis rendering.
    void on_cbx_RotationalSymmetries_currentIndexChanged(int index);         // Displaying a chosen rotational symmetry.
    void on_cbx_RenderLayer_currentIndexChanged(int index);                  // Selecting the layer to be displayed.

public:
    // CONSTRUCTOR AND DESTRUCTOR
    MainWindow(QWidget* parent = nullptr);  // Constructor of the main app window.
    ~MainWindow();                          // Destructor of the main app window.
};
#endif // MAINWINDOW_H
