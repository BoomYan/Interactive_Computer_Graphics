Assignemnt Task 2

Task 1: linFact and transFact
	Implement the linFact and transFact function in matrix4.h, which return linear and translational factor of an affine matrix.

Task 2: Views
	Cycle between 3 views (Sky camera view, cube1 view, cube2 view).

Task 3: Object Manipulation
	Manipulate the sky camera and the cubes. 
	Cycle between world-sky frame & sky-sky frame when the current camera view is sky view.
	Mouse Movement.

Task 4: Reset Scene
	Reset all to default.


This program is tested working on OS X 10.9.4 with OPENGL 2.x and GLSL 1.0.

In asst2.cpp:

Starting at line 74: Add global variables g_currentView, g_currentManipulatingObject, g_currentSkyView.
Starting at line 208: Add the second cube.
Starting at line 211: Add global variables g_auxFrame.
Starting at line 240: Initialize the second cube.
Starting at line 290: Set the eyeRbt.
Starting at line 318: Draw the second cube.
Starting at line 356: Set the g_auxFrame for transformation.
Starting at line 384: Update the g_skyRbt or g_objectRbt.
Starting at line 410: Realize the reset() function.
Starting at line 436: Add a function for setting current sky view.
Starting at line 450: Add a function for setting current view.
Starting at line 461: Add a function for setting current manipulating object.
Starting at line 495: Add case ‘v’ ‘o’ ‘m’ ‘r’.

In Matrix4.h:
Starting at line 284: Realize the transFact() function.
Starting at line 293: Realize the linFact() function.

Name: Jiaming Yan
Student ID: N14629568

Please feel free to contact me at any time:
jy1708@nyu.edu
