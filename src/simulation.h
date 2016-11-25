#ifndef SIMULATION_H
#define SIMULATION_H

#include "box.h"
#include "triangle.h"
#include "edge.h"
#include "camera.h"

class Simulation {
    // Window parameters
    int width;
    int height;

    // Different test scenes
    Box b1;
    Box b2;

    // Simulation state
    Camera c;
    PointMass* closest;
    bool frozen;
    bool chosen;
    float chosenDist;
    int mode; // 0 - view, 1 - picking
    bool gravity;
    float gForce;

  public:
    Simulation();
    void start();
    void keyHandler(unsigned char key, int x, int y);
    void mousePassiveHandler(int x, int y);
    void mousePressedHandler(int x, int y);
    void mouseButtonHandler(int button, int state, int x, int y);
    void displayFunc();

  private:
    void simulate();
};

#endif
