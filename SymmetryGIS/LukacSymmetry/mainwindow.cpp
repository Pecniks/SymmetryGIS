#include <chrono>
#include <QFileDialog>
#include <QMessageBox>

#include "camera/cameraparameters.h"
#include "mainwindow.h"
#include "point_reader/pointreader.h"
#include "ui_mainwindow.h"



// SLOTS
// Displaying the basic data about the app.
void MainWindow::on_actionAbout_triggered() {
    // Creating and displaying a message
    // box with the basic data.
    QMessageBox msg(
        QMessageBox::Information,
        "Authors",
        "GeMMA\n2021-22\n\nLuka Lukač\nDavid Podgorelec\nMatic Rašl",
        QMessageBox::Close
    );
    msg.exec();
}

// Resetting the camera position and rotation to initial values.
void MainWindow::on_actionResetCamera_triggered() {
    ui->openGLWidget->resetCamera();
}

// Camera top-down view according to the bounding box.
void MainWindow::on_actionTop_down_view_triggered() {
    const VoxelMesh& vm = LocalSymmetry::getVoxelMesh();  // Getting the voxel mesh.

    // Setting camera parameters.
    ui->openGLWidget->resetCamera();
    ui->openGLWidget->moveCameraToPoint((vm.minX + vm.maxX) / 2, (vm.minY + vm.maxY) / 2, -vm.maxZ - 50);
    ui->openGLWidget->rotateCamera(-90, 0, 0);
}

// Checking the camera position.
void MainWindow::on_actionCameraParameters_triggered() {
    auto params = ui->openGLWidget->getCameraParameters();  // Getting current camera parameters.

    // Execution of the form with camera parameters.
    CameraParameters c(params);
    // If clicked OK, the new camera parameters are set.
    if (c.exec()) {
        // Getting new camera parameters.
        const auto newParams = c.getParams();
        const float newFOV = std::get<0>(newParams);
        const std::vector<float> newPositions = std::get<1>(newParams);
        const std::vector<float> newRotations = std::get<2>(newParams);

        // Setting new camera parameters.
        ui->openGLWidget->resetCamera();
        ui->openGLWidget->setFOV(newFOV);
        ui->openGLWidget->moveCameraToPoint(newPositions[0], newPositions[1], -newPositions[2]);
        ui->openGLWidget->rotateCamera(newRotations[1], -newRotations[0], 0);
    }
}

// Function for choosing a LAS file.
void MainWindow::on_btn_ChooseFile_clicked() {
    // Creating a QFileDialog that accepts LAS files only.
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilter(tr("LAS files (*.las)"));

    // Dialog start.
    if (dialog.exec()) {
        // Reading the path to the file from the dialog.
        if (!dialog.selectedFiles().empty()) {
            ui->tbx_FileName->setText(dialog.selectedFiles().at(0));
            ui->btn_LoadPoints->setEnabled(true);
        }
    }
}

// Loading and rendering points.
void MainWindow::on_btn_LoadPoints_clicked() {
    // Reading points.
    const std::string path = ui->tbx_FileName->text().toStdString();                              // Getting the path to the LAS file.
    std::vector<Point<float>> pointsLAS = PointReader::readPointsFromLAS(path);                   // Reading points from the LAS file.
    LocalSymmetry::setPoints(pointsLAS);

    // Calculating a voxel mesh and rendering points.
    const VoxelMesh& voxelMesh = LocalSymmetry::getVoxelMesh();                                   // Getting voxel mesh to find extreme points.
    const std::vector<Point<float>>& points = LocalSymmetry::getPoints();                         // Getting LAS points.
    const std::vector<PositionFromPlane> positions(points.size(), PositionFromPlane::undefined);  // Positions from a non-existent plane in the beginning are undefined.
    ui->openGLWidget->setPoints(points, positions);                                               // Rendering points in OpenGL.
    ui->openGLWidget->setVoxelMesh(voxelMesh);                                                    // Voxel mesh setting in OpenGL.
    ui->openGLWidget->setRenderSymmetryAxis(false);                                               // Disabling symmetry axis rendering in OpenGL.
    ui->openGLWidget->setRenderSymmetryPlane(false);                                              // Disabling symmetry plane rendering in OpenGL.

    // Basic description dump in the console.
    ui->tbx_Console->moveCursor(QTextCursor::End);
    ui->tbx_Console->setTextColor(QColor(0, 100, 0));
    ui->tbx_Console->insertPlainText("POINT LOADING\n");
    ui->tbx_Console->setTextColor(QColor(0, 0, 0));
    ui->tbx_Console->insertPlainText("Number of points: " + QString::number(LocalSymmetry::getPoints().size()) + "\n");
    ui->tbx_Console->insertPlainText("minX: " + QString::number(voxelMesh.minX) + "\n");
    ui->tbx_Console->insertPlainText("maxX: " + QString::number(voxelMesh.maxX) + "\n");
    ui->tbx_Console->insertPlainText("minY: " + QString::number(voxelMesh.minY) + "\n");
    ui->tbx_Console->insertPlainText("maxY: " + QString::number(voxelMesh.maxY) + "\n");
    ui->tbx_Console->insertPlainText("minZ: " + QString::number(voxelMesh.minZ) + "\n");
    ui->tbx_Console->insertPlainText("maxZ: " + QString::number(voxelMesh.maxZ) + "\n");
    ui->tbx_Console->insertPlainText("deltaX: " + QString::number(voxelMesh.deltaX) + "\n");
    ui->tbx_Console->insertPlainText("deltaY: " + QString::number(voxelMesh.deltaY) + "\n");
    ui->tbx_Console->insertPlainText("deltaZ: " + QString::number(voxelMesh.deltaZ) + "\n\n");
    ui->tbx_Console->moveCursor(QTextCursor::End);

    // Setting controls enabled or disabled.
    ui->actionTop_down_view->setEnabled(true);
    ui->gbx_Voxelization->setEnabled(true);
    ui->gbx_InterestingVoxelsSearch->setEnabled(false);
    ui->gbx_ReflectionSymmetriesFinding->setEnabled(false);
    ui->gbx_RotationalSymmetriesFinding->setEnabled(false);
    ui->gbx_RenderProjection->setEnabled(true);
    ui->gbx_PointVisualization->setEnabled(true);
    ui->gbx_VoxelVisualization->setEnabled(false);
    ui->gbx_RenderReflectionSymmetries->setEnabled(false);
    ui->gbx_RenderRotationalSymmetries->setEnabled(false);
    ui->gbx_LayerVisualization->setEnabled(false);

    // Moving the camera to the start of the LAS bounding box.
    ui->openGLWidget->moveCameraToPoint(voxelMesh.minX, voxelMesh.minY, voxelMesh.minZ);
}

// Selecting voxel calculation by side length.
void MainWindow::on_rb_VoxelizationSideLength_clicked() {
    ui->swgt_VoxelizationMethod->setCurrentIndex(0);  // Displaying controls for voxelization by voxel side length.
}

// Selecting voxel calculation by maximum count.
void MainWindow::on_rb_VoxelizationCount_clicked() {
    ui->swgt_VoxelizationMethod->setCurrentIndex(1);  // Displaying controls for voxelization by maximum voxel count.
}

// Voxel mesh calculation.
void MainWindow::on_btn_CalculateVoxels_clicked() {
    // Calculating voxel mesh by voxel side length.
    if (ui->swgt_VoxelizationMethod->currentIndex() == 0) {
        const int sideLength = ui->sbx_VoxelSideLength->value();         // Reading the user given voxel side length.
        LocalSymmetry::calculateVoxelMeshByVoxelSideLength(sideLength);  // Calculating the voxel mesh by maximum voxel count.
    }
    // Calculating voxel mesh by maximum voxel count.
    else {
        const int numberOfVoxels = ui->sbx_VoxelCount->value();                // Reading the user given maximum number of voxels.
        LocalSymmetry::calculateVoxelMeshByMaximumVoxelCount(numberOfVoxels);  // Calculating the voxel mesh by maximum voxel count.
    }

    // Voxel rendering in OpenGL.
    const VoxelMesh& voxelMesh = LocalSymmetry::getVoxelMesh();  // Getting voxel mesh.
    auto voxels = LocalSymmetry::getVoxels();                    // Getting voxels from the voxel mesh.
    auto points = LocalSymmetry::getPoints();                    // Getting LAS points.
    ui->openGLWidget->setVoxels(voxels, voxelMesh, std::vector<PositionFromPlane>(voxelMesh.voxelCount, PositionFromPlane::undefined));  // Setting points and rendering in OpenGL.
    ui->openGLWidget->setRenderSymmetryAxis(false);              // Disabling symmetry axis rendering in OpenGL.
    ui->openGLWidget->setRenderSymmetryPlane(false);             // Disabling symmetry plane rendering in OpenGL.
    ui->openGLWidget->setPoints(points, std::vector<PositionFromPlane>(points.size(), PositionFromPlane::undefined));

    if (voxelMesh.voxelCount > 0) {
        // Printing data in the console.
        ui->tbx_Console->moveCursor(QTextCursor::End);
        ui->tbx_Console->setTextColor(QColor(0, 100, 0));
        ui->tbx_Console->insertPlainText("VOXEL MESH CALCULATION\n");
        ui->tbx_Console->setTextColor(QColor(0, 0, 0));
        ui->tbx_Console->insertPlainText("Number of voxels: " + QString::number(voxelMesh.voxelCount) + "\n");
        ui->tbx_Console->insertPlainText("Voxel side length: " + QString::number(voxelMesh.voxelSideSize) + "\n");
        ui->tbx_Console->insertPlainText("Voxels by X: " + QString::number(voxelMesh.voxelX) + "\n");
        ui->tbx_Console->insertPlainText("Voxels by Y: " + QString::number(voxelMesh.voxelY) + "\n");
        ui->tbx_Console->insertPlainText("Voxels by Z: " + QString::number(voxelMesh.voxelZ) + "\n\n");
        ui->tbx_Console->moveCursor(QTextCursor::End);

        // Adding all layers to the combo box (rendering by layer).
        ui->cbx_RenderLayer->clear();                                   // Clearing obsolete layers in the combo box.
        ui->cbx_RenderLayer->addItem(QString::fromUtf8("All layers"));  // Adding option to render all layers.
        for (int i = 0; i < voxelMesh.voxelZ; i++) {
            ui->cbx_RenderLayer->addItem(QString::number(i));           // Adding each layer.
        }
        ui->cbx_RenderLayer->setEnabled(true);                          // Enabling the combo box for selecting layers.

        // Enabling/disabling group boxes.
        ui->gbx_Voxelization->setEnabled(true);
        ui->gbx_InterestingVoxelsSearch->setEnabled(true);
        ui->gbx_ReflectionSymmetriesFinding->setEnabled(false);
        ui->gbx_RotationalSymmetriesFinding->setEnabled(false);
        ui->gbx_RenderProjection->setEnabled(true);
        ui->gbx_PointVisualization->setEnabled(true);
        ui->gbx_VoxelVisualization->setEnabled(true);
        ui->gbx_RenderReflectionSymmetries->setEnabled(false);
        ui->gbx_RenderRotationalSymmetries->setEnabled(false);
        ui->gbx_LayerVisualization->setEnabled(true);
    }
}

// Finding interesting voxels.
void MainWindow::on_btn_FindInterestingVoxels_clicked() {
    // Getting structures for finding interesting voxels.
    const VoxelMesh& voxelMesh = LocalSymmetry::getVoxelMesh();                      // Getting a voxel mesh.
    const int minClusterSize = ui->sbx_InterestingVoxelMinimumClusterSize->value();  // Getting a minimum cluster size.

    // Interesting and super-interesting voxels search procedure.
    int superInterestingCount = 0;
    int interestingCount = 0;
    LocalSymmetry::findInterestingVoxels(minClusterSize, superInterestingCount, interestingCount);

    // Force rerendering in OpenGL.
    auto& voxels = LocalSymmetry::getVoxels();  // Getting a voxel vector.
    ui->openGLWidget->setVoxels(voxels, voxelMesh, std::vector<PositionFromPlane>(voxelMesh.voxelCount, PositionFromPlane::undefined));  // Setting points and rendering in OpenGL.
    ui->openGLWidget->update();
    ui->btn_FindRotationalSymmetries->setEnabled(true);
    ui->btn_FindReflectionSymmetries->setEnabled(true);

    // Data output in the console.
    ui->tbx_Console->moveCursor(QTextCursor::End);
    ui->tbx_Console->setTextColor(QColor(0, 100, 0));
    ui->tbx_Console->insertPlainText("INTERESTING VOXEL SEARCH PROCEDURE\n");
    ui->tbx_Console->setTextColor(QColor(0, 0, 0));
    ui->tbx_Console->insertPlainText("Number of super-interesting voxels: " + QString::number(superInterestingCount) + "\n");
    ui->tbx_Console->insertPlainText("Number of interesting voxels: " + QString::number(interestingCount) + "\n\n");
    ui->tbx_Console->moveCursor(QTextCursor::End);

    // Enabling/disabling group boxes.
    ui->gbx_Voxelization->setEnabled(true);
    ui->gbx_InterestingVoxelsSearch->setEnabled(true);
    ui->gbx_ReflectionSymmetriesFinding->setEnabled(true);
    ui->gbx_RotationalSymmetriesFinding->setEnabled(true);
    ui->gbx_RenderProjection->setEnabled(true);
    ui->gbx_PointVisualization->setEnabled(true);
    ui->gbx_VoxelVisualization->setEnabled(true);
    ui->gbx_RenderReflectionSymmetries->setEnabled(false);
    ui->gbx_RenderRotationalSymmetries->setEnabled(false);
    ui->gbx_LayerVisualization->setEnabled(true);
}

// Reflection symmetry search procedure.
void MainWindow::on_btn_FindReflectionSymmetries_clicked() {
    const double tolerance = ui->cbx_ReflectionSymmetriesTolerance->value();  // Algorithm tolerance.
    auto& voxels = LocalSymmetry::getVoxels();

    // Retrieving interesting voxel points.
    VoxelMesh voxelMesh = LocalSymmetry::getVoxelMesh();
    std::vector<Point<float>> points = Voxel::getInterestingPointsFromVoxels(
        voxels,
        voxelMesh.voxelSideSize
    );

    // Finding reflection symmetries.
    const int minimumClusterSize = ui->cbx_MinimumClusterSize->value();
    auto timeStart = std::chrono::steady_clock::now();
    reflectionSymmetries = ReflectionSymmetry::calculateReflectionSymmetriesToResult(points, tolerance, minimumClusterSize);
    auto timeEnd = std::chrono::steady_clock::now();

    // Finding partial symmetries.
    const int minimumPartialClusterSize = ui->cbx_MinimumPartialClusterSize->value();
    auto timeStartPartial = std::chrono::steady_clock::now();
//    partialReflectionSymmetries = ReflectionSymmetry::findPartialSymmetries(reflectionSymmetries, minimumPartialClusterSize);
    auto timeEndPartial = std::chrono::steady_clock::now();

    // Time calculation.
    const int milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();
    const int millisecondsPartial = std::chrono::duration_cast<std::chrono::milliseconds>(timeEndPartial - timeStartPartial).count();
    const int total = milliseconds + millisecondsPartial;

    if (reflectionSymmetries.size() > 0) {
        // Adding all reflection symmetries to the combobox.
        ui->cbx_ReflectionSymmetries->clear();
        for (unsigned long long i = 0; i < partialReflectionSymmetries.size(); i++) {
            Plane plane = partialReflectionSymmetries[i].getPlane();
            QString str = "(" + QString::number(i) + ")  " + QString::fromStdString(plane.toString());
            ui->cbx_ReflectionSymmetriesPartial->addItem(str);
        }
        // Adding all reflection symmetries to the combobox.
        for (unsigned long long i = 0; i < reflectionSymmetries.size(); i++) {
//            Plane plane = reflectionSymmetries[i].getPlane();
//            QString str = "(" + QString::number(i) + ")  " + QString::fromStdString(plane.toString());
//            ui->cbx_ReflectionSymmetries->addItem(str);
        }
        ui->cbx_ReflectionSymmetries->setEnabled(true);
        ui->cbx_RenderLineSegments->setEnabled(true);
        ui->cbx_RenderSymmetryVoxels->setEnabled(true);
        ui->openGLWidget->setRenderSymmetryPlane(true);

        // Data output in the console.
        ui->tbx_Console->moveCursor(QTextCursor::End);
        ui->tbx_Console->setTextColor(QColor(0, 100, 0));
        ui->tbx_Console->insertPlainText("REFLECTION SYMMETRIES SEARCH PROCEDURE\n");
        ui->tbx_Console->setTextColor(QColor(0, 0, 0));
        ui->tbx_Console->insertPlainText("Number of reflection symmetries: " + QString::number(reflectionSymmetries.size()) + "\n");
        ui->tbx_Console->insertPlainText("Number of partial reflection symmetries: " + QString::number(partialReflectionSymmetries.size()) + "\n");
        ui->tbx_Console->insertPlainText("Elapsed time for local symmetries: " + QString::number(milliseconds) + " ms\n");
        ui->tbx_Console->insertPlainText("Elapsed time for partial symmetries: " + QString::number(millisecondsPartial) + " ms\n");
        ui->tbx_Console->insertPlainText("Total elapsed time: " + QString::number(total) + " ms\n\n");
        ui->tbx_Console->moveCursor(QTextCursor::End);

        // Enabling/disabling group boxes.
        ui->gbx_Voxelization->setEnabled(true);
        ui->gbx_InterestingVoxelsSearch->setEnabled(false);
        ui->gbx_ReflectionSymmetriesFinding->setEnabled(true);
        ui->gbx_RotationalSymmetriesFinding->setEnabled(true);
        ui->gbx_RenderProjection->setEnabled(true);
        ui->gbx_PointVisualization->setEnabled(true);
        ui->gbx_VoxelVisualization->setEnabled(true);
        ui->gbx_RenderReflectionSymmetries->setEnabled(true);
        ui->gbx_RenderRotationalSymmetries->setEnabled(false);
        ui->gbx_LayerVisualization->setEnabled(true);

        ui->openGLWidget->setRenderSymmetryAxis(false);  // Disabling symmetry axis rendering in OpenGL.
    }
}

// Rotational symmetries search procedure (brute force).
void MainWindow::on_btn_FindRotationalSymmetriesBrute_clicked() {
    // Getting a voxel mesh and points.
    VoxelMesh& voxelMesh = ui->openGLWidget->getVoxelMesh();

    // Time measuring start.
    auto timeStart = std::chrono::steady_clock::now();

    // Calculating 3D voxel vector and rotational symmetries.
    auto normalisedVoxels = rotationalSymmetry.getNormalisedVoxelVector();
    rotationalSymmetries = rotationalSymmetry.getRotationalSymmetries(normalisedVoxels, voxelMesh);

    // Time measuring end.
    auto timeEnd = std::chrono::steady_clock::now();
    const int milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();

    // Adding all rotational symmetries to the combobox.
    for (unsigned long long i = 0; i < rotationalSymmetries.size(); i++) {
        QString str = (
            "(" +
            QString::number(i) +
            ")  Level " + QString::number(rotationalSymmetries[i].getRotation()) +
            ", Axis (" + QString::number(rotationalSymmetries[i].getSymmetryAxis().getX()) +
            ", " + QString::number(rotationalSymmetries[i].getSymmetryAxis().getY()) + ")"
        );
        ui->cbx_RotationalSymmetries->addItem(str);
    }

    // Enabling combobox with rotational symmetries.
    if (rotationalSymmetries.size() > 0) {
        // Data output in the console.
        ui->tbx_Console->moveCursor(QTextCursor::End);
        ui->tbx_Console->setTextColor(QColor(0, 100, 0));
        ui->tbx_Console->insertPlainText("ROTATIONAL SYMMETRIES SEARCH PROCEDURE\n");
        ui->tbx_Console->setTextColor(QColor(0, 0, 0));
        ui->tbx_Console->insertPlainText("Number of rotational symmetries: " + QString::number(rotationalSymmetries.size()) + "\n");
        ui->tbx_Console->insertPlainText("Maximum symmetry level: " + QString::number(rotationalSymmetries[0].getRotation()) + "\n");
        ui->tbx_Console->insertPlainText("Elapsed time: " + QString::number(milliseconds) + " ms\n\n");
        ui->tbx_Console->moveCursor(QTextCursor::End);

        ui->gbx_RenderRotationalSymmetries->setEnabled(true);
        ui->openGLWidget->setRenderSymmetryPlane(false);  // Disabling symmetry axis rendering in OpenGL.
    }
    else {
        QMessageBox msg(QMessageBox::Warning,
                        "Warning",
                        "No rotational symmetries found.",
                        QMessageBox::Close);
        msg.exec();
        return;
    }
}

// Rotational symmetries search procedure (wise).
void MainWindow::on_btn_FindRotationalSymmetries_clicked() {
    auto& voxels = LocalSymmetry::getVoxels();

    // Retrieving tolerances from the GUI.
    const double lengthTolerance = ui->sbx_RotationalLengthTolerance->value();
    const double angleTolerance = ui->sbx_RotationalAngleTolerance->value();

    // Retrieving interesting voxel points.
    VoxelMesh voxelMesh = LocalSymmetry::getVoxelMesh();
//    std::vector<Point<float>> points = Voxel::getInterestingPointsFromVoxels(
//        voxels,
//        voxelMesh.voxelSideSize
//    );
    const std::vector<Point<float>>& points = LocalSymmetry::getPoints();

    // Finding reflection symmetries.
    auto timeStart = std::chrono::steady_clock::now();
    rotationalSymmetriesWise = RotationalSymmetry::calculateRotationalSymmetries(points, lengthTolerance, angleTolerance);
    auto timeEnd = std::chrono::steady_clock::now();

    // Time calculation.
    const int milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart).count();

    // Data output in the console.
    ui->tbx_Console->moveCursor(QTextCursor::End);
    ui->tbx_Console->setTextColor(QColor(0, 100, 0));
    ui->tbx_Console->insertPlainText("ROTATIONAL SYMMETRIES SEARCH PROCEDURE\n");
    ui->tbx_Console->setTextColor(QColor(0, 0, 0));
    ui->tbx_Console->insertPlainText("Number of rotational symmetries: " + QString::number(reflectionSymmetries.size()) + "\n");
    ui->tbx_Console->insertPlainText("Elapsed time for rotational symmetries: " + QString::number(milliseconds) + " ms\n\n");
    ui->tbx_Console->moveCursor(QTextCursor::End);

    ui->openGLWidget->setRenderSymmetryPlane(false);  // Disabling symmetry axis rendering in OpenGL.
    ui->openGLWidget->setRenderSymmetryAxis(true);    // Disabling symmetry axis rendering in OpenGL.
}




// Setting perspective OpenGL projection.
void MainWindow::on_rb_PerspectiveProjection_clicked() {
    ui->openGLWidget->setRenderPerspectiveProjection(true);
}

// Setting orthogonal OpenGL projection.
void MainWindow::on_rb_OrthoProjection_clicked() {
    ui->openGLWidget->setRenderPerspectiveProjection(false);
}

// Enabling/disabling bounding box rendering in OpenGL.
void MainWindow::on_cbx_RenderBoundingBox_stateChanged(int arg1) {
    ui->openGLWidget->setRenderBoundingBox((bool)arg1);
}

// Enabling/disabling point rendering.
void MainWindow::on_cbx_RenderPoints_stateChanged(int arg1) {
    ui->openGLWidget->setRenderPoints((bool)arg1);
}

// Rendering voxels as edges.
void MainWindow::on_rb_RenderVoxelEdges_clicked() {
    ui->openGLWidget->setRenderVoxelsAsEdges(true);
}

// Rendering voxels as cubes.
void MainWindow::on_rb_RenderVoxelCubes_clicked() {
    ui->openGLWidget->setRenderVoxelsAsEdges(false);
}

// Enabling/disabling voxel rendering in OpenGL.
void MainWindow::on_cbx_RenderVoxels_stateChanged(int arg1) {
    ui->openGLWidget->setRenderVoxels((bool)arg1);

    // Enabling/disabling of the dependent checkbox.
    if (!arg1) {
        ui->cbx_RenderOrdinaryVoxels->setEnabled(false);
        ui->cbx_RenderSuperInterestingVoxels->setEnabled(false);
        ui->cbx_RenderInterestingVoxels->setEnabled(false);
        ui->cbx_RenderSymmetryVoxels->setEnabled(false);
    }
    else {
        ui->cbx_RenderOrdinaryVoxels->setEnabled(true);
        ui->cbx_RenderSuperInterestingVoxels->setEnabled(true);
        ui->cbx_RenderInterestingVoxels->setEnabled(true);
        ui->cbx_RenderSymmetryVoxels->setEnabled(true);
    }
}

// Enabling/disabling ordinary voxels rendering in OpenGL.
void MainWindow::on_cbx_RenderOrdinaryVoxels_stateChanged(int arg1) {
    ui->openGLWidget->setRenderOrdinaryVoxels((bool)arg1);
}

// Enabling/disabling super-interesting voxels rendering in OpenGL.
void MainWindow::on_cbx_RenderSuperInterestingVoxels_stateChanged(int arg1) {
    ui->openGLWidget->setRenderSuperInterestingVoxels((bool)arg1);
}

// Enabling/disabling interesting voxels rendering in OpenGL.
void MainWindow::on_cbx_RenderInterestingVoxels_stateChanged(int arg1) {
    ui->openGLWidget->setRenderInterestingVoxels((bool)arg1);
}

// Enabling/disabling symmetry voxels rendering in OpenGL.
void MainWindow::on_cbx_RenderSymmetryVoxels_stateChanged(int arg1) {
    ui->openGLWidget->setRenderSymmetryVoxels((bool)arg1);
}

// Enabling/disabling symmetry plane rendering.
void MainWindow::on_cbx_RenderSymmetryPlane_stateChanged(int arg1) {
    ui->openGLWidget->setRenderSymmetryPlane((bool)arg1);
}

// Enabling/disabling line segment rendering.
void MainWindow::on_cbx_RenderLineSegments_stateChanged(int arg1) {
    ui->openGLWidget->setRenderLineSegments((bool)arg1);
}

// Local reflection symmetries list display.
void MainWindow::on_rb_VisualizationLocalReflectionSymmetries_clicked() {
    ui->swgt_ReflectionSymmetryType->setCurrentIndex(0);
    on_cbx_ReflectionSymmetries_currentIndexChanged(ui->cbx_ReflectionSymmetries->currentIndex());
}

// Partial reflection symmetries list display.
void MainWindow::on_rb_VisualizationPartialReflectionSymmetries_clicked() {
    ui->swgt_ReflectionSymmetryType->setCurrentIndex(1);
    on_cbx_ReflectionSymmetriesPartial_currentIndexChanged(ui->cbx_ReflectionSymmetriesPartial->currentIndex());
}

// Displaying a chosen reflection symmetry.
void MainWindow::on_cbx_ReflectionSymmetries_currentIndexChanged(int index) {
    if (index != -1) {
        // Getting line segments and the plane.
//        std::vector<LineSegment> lineSegments = reflectionSymmetries[index].getLineSegments();
//        const Plane plane = reflectionSymmetries[index].getPlane();

//        // Setting and rendering points in OpenGL.
//        const std::vector<Point<float>>& points = LocalSymmetry::getPoints();                                                       // Getting LAS points.
//        const std::vector<PositionFromPlane>& positions = reflectionSymmetries[index].calculatePositionsFromPlane(points, &plane);  // Positions in the beginning are neutral.
//        ui->openGLWidget->setPoints(points, positions);
//        ui->openGLWidget->setLineSegments(lineSegments, plane);

//        // Voxel resetting.
//        std::vector<Voxel> voxels = reflectionSymmetries[index].getVoxels();
//        std::vector<Voxel>& openGLvoxels = ui->openGLWidget->getVoxels();
//        VoxelMesh voxelMesh = LocalSymmetry::getVoxelMesh();
//        for (Voxel& voxel : openGLvoxels) {
//            voxel.setInSymmetry(false);
//        }

//        // Visualization of a voxel in the symmetry.
//        for (const Voxel& v : voxels) {
//            int index = v.getZ() * voxelMesh.voxelY * voxelMesh.voxelX +
//                        v.getY() * voxelMesh.voxelX +
//                        v.getX();

//            openGLvoxels[index].setInSymmetry(true);
//        }

//        // Calculating the voxel positions.
//        std::vector<Point<float>> voxelPoints = Voxel::getInterestingPointsFromVoxels(LocalSymmetry::getVoxels(), voxelMesh.voxelSideSize, false);
//        std::vector<PositionFromPlane> voxelPositions = reflectionSymmetries[index].calculatePositionsFromPlane(voxelPoints, &plane);
//        ui->openGLWidget->setVoxelPositions(voxelPositions);

        ui->openGLWidget->update();
    }
}

// Displaying a chosen partial reflection symmetry.
void MainWindow::on_cbx_ReflectionSymmetriesPartial_currentIndexChanged(int index) {
    if (index != -1) {
        // Getting line segments and the plane.
        std::vector<LineSegment> lineSegments = partialReflectionSymmetries[index].getLineSegments();
        const Plane plane = partialReflectionSymmetries[index].getPlane();

        // Setting and rendering points in OpenGL.
        const std::vector<Point<float>>& points = LocalSymmetry::getPoints();                                                              // Getting LAS points.
        const std::vector<PositionFromPlane>& positions = partialReflectionSymmetries[index].calculatePositionsFromPlane(points,& plane);  // Positions in the beginning are neutral.
        ui->openGLWidget->setPoints(points, positions);
        ui->openGLWidget->setLineSegments(lineSegments, plane);

        // Voxel resetting.
        std::vector<Voxel> voxels = partialReflectionSymmetries[index].getVoxels();
        std::vector<Voxel>& openGLvoxels = ui->openGLWidget->getVoxels();
        VoxelMesh voxelMesh = LocalSymmetry::getVoxelMesh();
        for (Voxel& voxel : openGLvoxels) {
            voxel.setInSymmetry(false);
        }

        // Visualization of a voxel in the symmetry.
        for (const Voxel& v : voxels) {
            int index = v.getZ() * voxelMesh.voxelY * voxelMesh.voxelX +
                        v.getY() * voxelMesh.voxelX +
                        v.getX();

            openGLvoxels[index].setInSymmetry(true);
        }

        // Calculating the voxel positions.
        std::vector<Point<float>> voxelPoints = Voxel::getInterestingPointsFromVoxels(LocalSymmetry::getVoxels(), voxelMesh.voxelSideSize, false);
        std::vector<PositionFromPlane> voxelPositions = partialReflectionSymmetries[index].calculatePositionsFromPlane(voxelPoints, &plane);
        ui->openGLWidget->setVoxelPositions(voxelPositions);

        ui->openGLWidget->update();
    }
}

// Moving the camera to the top of the reflection symmetry.
void MainWindow::on_btn_ReflectionSymmetryTopView_clicked() {
    // Getting the basic data.
//    const VoxelMesh& voxelMesh = LocalSymmetry::getVoxelMesh();
//    const int index = ui->cbx_ReflectionSymmetries->currentIndex();
//    const Plane plane = reflectionSymmetries[index].getPlane();
//    std::tuple<Vector3d, double, double, double> planeParams = Plane::calculateStartPoint(plane, voxelMesh);
//    Vector3d start = std::get<0>(planeParams);
//    double lengthX = std::get<2>(planeParams);
//    double lengthY = std::get<3>(planeParams);

//    // Moving camera to the symmetry.
//    ui->openGLWidget->resetCamera();
//    if (plane.calculateParallelVector().getY() < 0) {
//        ui->openGLWidget->moveCameraToPoint(start.getX() + lengthX, start.getY() - lengthY, -voxelMesh.maxZ - 50);
//    }
//    else {
//        ui->openGLWidget->moveCameraToPoint(start.getX() + lengthX, start.getY() + lengthY, -voxelMesh.maxZ - 50);
//    }
//    ui->openGLWidget->rotateCamera(-90.0f, 0.0f, 0.0f);
    ui->openGLWidget->update();
}

// Enabling/disabling symmetry axis rendering.
void MainWindow::on_cbx_RenderSymmetryAxis_stateChanged(int arg1) {
    ui->openGLWidget->setRenderSymmetryAxis((bool)arg1);
}

// Displaying a chosen rotational symmetry.
void MainWindow::on_cbx_RotationalSymmetries_currentIndexChanged(int index) {
    // Getting the basic data about the symmetry.
    std::vector<Voxel> voxels = rotationalSymmetries[index].getVoxels();
    std::vector<Voxel>& openGLvoxels = ui->openGLWidget->getVoxels();
    const VoxelMesh& voxelMesh = rotationalSymmetry.getVoxelMesh();
    const Point<double> axis = rotationalSymmetries[index].getSymmetryAxis();

    // Rendering of the symmetry in OpenGL.
    const std::vector<Point<float>>& points = LocalSymmetry::getPoints();                                              // Getting LAS points.
    const std::vector<PositionFromPlane>& positions = rotationalSymmetries[index].calculatePositionsFromAxis(&axis);
    ui->openGLWidget->setPoints(points, positions);

    // Voxel resetting.
    for (Voxel& voxel : openGLvoxels) {
        voxel.setInSymmetry(false);
    }

    // Visualization of a voxel in the symmetry.
    for (const Voxel& voxel : voxels) {
        // Voxel index calculation.
        int index = voxel.getZ() * voxelMesh.voxelY * voxelMesh.voxelX +
                    voxel.getY() * voxelMesh.voxelX +
                    voxel.getX();

        openGLvoxels[index].setInSymmetry(true);
    }

    // Moving camera to the symmetry.
    Point<double> symmetryAxis = rotationalSymmetries[index].getSymmetryAxis() * (double)voxelMesh.voxelSideSize;
    symmetryAxis.setX(symmetryAxis.getX() + voxelMesh.minX);
    symmetryAxis.setY(symmetryAxis.getY() + voxelMesh.minY);
    ui->openGLWidget->setSymmetryAxis(symmetryAxis, true);
    ui->openGLWidget->update();
}

// Selecting the layer to be displayed.
void MainWindow::on_cbx_RenderLayer_currentIndexChanged(int index) {
    ui->openGLWidget->setRenderLayer(index - 1);
    ui->openGLWidget->update();
}



// CONSTRUCTOR AND DESTRUCTOR
// Constructor of the main app window.
MainWindow::MainWindow(QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->gbx_Voxelization->setEnabled(false);
    ui->gbx_InterestingVoxelsSearch->setEnabled(false);
    ui->gbx_ReflectionSymmetriesFinding->setEnabled(false);
    ui->gbx_RotationalSymmetriesFinding->setEnabled(false);
    ui->gbx_RenderProjection->setEnabled(true);
    ui->gbx_PointVisualization->setEnabled(true);
    ui->gbx_VoxelVisualization->setEnabled(false);
    ui->gbx_RenderReflectionSymmetries->setEnabled(false);
    ui->gbx_RenderRotationalSymmetries->setEnabled(false);
    ui->gbx_LayerVisualization->setEnabled(false);
}

// Destructor of the main app window.
MainWindow::~MainWindow() {
    delete ui;
}
