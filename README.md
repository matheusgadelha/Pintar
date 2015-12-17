# Pintar
Small rendering engine with Eigen linear algebra data structures on top of OpenGL. Made for prototyping interactive graphics solutions. It currently contains:
* basic hierarchical object structure;
* camera and arcball camera definitions;
* mesh loading and (slow) traversal; and 
* rigid body simulation. 

The main program is a demo of the rigid body simulator where you can draw the force you want to apply to the model.

##Dependencies
This code requires that you have the static library of OOGL(https://github.com/Overv/OOGL/) inside the lib folder. Download it, compile it and copy the OOGL.a file to the lib folder of this project.

As the main description suggests, this code also requires Eigen installed.

##Compilation
$sh compile.sh

##Execution
./bin/main models/cube.obj
