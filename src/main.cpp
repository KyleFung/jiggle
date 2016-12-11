#include <GLUT/glut.h>
#include <stdlib.h>
#include <iostream>

#include "utils.h"
#include "simulation.h"

// Apple hack
#ifdef __APPLE__
#include <ApplicationServices/ApplicationServices.h>
#endif

Simulation s;

static void displayCallback() {
    s.displayFunc();
}

static void keyCallback(unsigned char key, int x, int y) {
    s.keyHandler(key, x, y);
}

static void mousePassiveCallback(int x, int y) {
    s.mousePassiveHandler(x, y);
}

static void mousePressedCallback(int x, int y) {
    s.mousePressedHandler(x, y);
}

static void mouseButtonCallback(int button, int state, int x, int y) {
    s.mouseButtonHandler(button, state, x, y);
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("Squishy");

    glClearColor(0, 0, 0, 0);
    glEnable(GL_DEPTH_TEST);

    // Set up callbacks
    glutDisplayFunc(displayCallback);
    glutIdleFunc(displayCallback);
    glutKeyboardFunc(keyCallback);
    glutPassiveMotionFunc(mousePassiveCallback);
    glutMotionFunc(mousePressedCallback);
    glutMouseFunc(mouseButtonCallback);

    // Apple hack
    #ifdef __APPLE__
    CGSetLocalEventsSuppressionInterval(0.0);
    #endif

    s.start();
}
