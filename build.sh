#! /bin/bash
g++ -I thirdparty/eigen -w src/*.cpp -framework GLUT -framework OpenGL -framework ApplicationServices -o jiggle.out
