# Pintar
Small rendering engine with Eigen linear algebra data structures on top of OpenGL. Made for prototyping interactive graphics solutions. It currently contains:
* basic hierarchical object structure;
* camera and arcball camera definitions;
* mesh loading and (slow) traversal; and 
* rigid body simulation. 

The main program is a demo of the rigid body simulator where you can draw the force you want to apply to the model.
This program requires that you have the static library of OOGL(https://github.com/Overv/OOGL/) inside the lib folder.

##Compilation
$sh compile.sh

##Execution
./bin/main models/cube.obj
