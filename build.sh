#! /bin/bash
g++ -I eigen -w src/*.cpp -framework GLUT -framework OpenGL -framework ApplicationServices -o jiggle.out
