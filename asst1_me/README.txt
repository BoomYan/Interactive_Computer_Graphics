Assignemnt Task 1

Task 1:
Modify the program so that when the window is reshaped, the aspect ratio of the object drawn does not change.

Task 2:
Modify the fragment shader so that the image automatically interpolates between the squirrel and the smiley. 


This program is tested working on OS X 10.9.4 with OPENGL 2.x and GLSL 1.0.

In asst1.cpp:

Starting at line 70: Add global variables g_uScaleX and g_uScaleY.

Starting at line 81: Add handles h_uScaleX and h_uScaleY.

Starting at line 285: Calculate the g_uScaleX and g_uScaleY in the function reShape() in order to preserve the ratio.

In asst1-gl2.vshader:

Starting at line 3: Add uniform variables uScaleX and uScaleY.

Starting at line 16: Modify the vertex coordinate using the variables above.

Starting at line 15: Introduce the lerper to show the rotation effect.

In asst1-gl2.fshader:

Starting at line 14: Modify the lerper to perform the automatic interpolation.

Name: Jiaming Yan
Student ID: N14629568
