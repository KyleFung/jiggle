# jiggle
An interactive simulation of deformable solids using a mass-spring model. This application uses a semi-implicit time stepping scheme to ensure stable simulation of Hookean springs. Continuous collision detection is implemented to ensure objects do not pass through each other, and is accelerated using a bounding volume hierarchy.

<b>In progress:</b>  
Generating response impulses when collisions are detected

<b>Future features:</b>  
Fast mass springs (http://graphics.berkeley.edu/papers/Liu-FSM-2013-11/Liu-FSM-2013-11.pdf)  
Volume preservation via cross springs

<b>References:</b>  
Spring dynamics (https://www.cs.cmu.edu/~baraff/papers/sig98.pdf)  
Continuous collision detection (https://graphics.stanford.edu/courses/cs468-02-winter/Papers/Collisions_vetements.pdf)  
Collision response (https://www.cs.utah.edu/~ladislav/kavan03rigid/kavan03rigid.pdf)

<b>Dependencies:</b>  
OpenGL (You will need a copy of the OpenGL dll/lib)   
GLUT (You will need to get a copy of the glut and glu dll/lib)  
Eigen (Included as a submodule)

Note: This project is built on Windows, but a script for building on Mac is included (build.sh)
