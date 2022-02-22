This folder contains the implementation of the basic version (without using the weights in the symmetry measure) of the proposed method which detects the most significant symmetry plane of a given 3D object.

Usage: SymmetryDetector <file.obj/pc> <visualize>
<file.obj/pc> - input object in OBJ format or as a point cloud (see below)
<visualize> - determines whether visualization window will be opened after the program finishes:
	true - visualization window will be opened
	false (default) - visualization window will not be opened
	for the visualization window to open DirectX needs to be installed
	visualization controls:
		- click and move the mouse to rotate the object
		- hold L and move the mouse to move the light around
		- press Up and Down arrows to rotate the object around the axis perpendicular to the symmetry plane
The outputed time does not include loading the input object
The coefficients of the detected plane are stored in a file with the same name as the input file and .txt extension added in the following format: a;b;c;d


Apart from OBJ files the program can process point clouds with .pc extension in the following format:
N
x1 y1 z1
x2 y2 z2
.
.
.
xN yN zN
where N is the point count and xi, yi and zi are the coordinates of i-th point

Test objects are in the data folder
