#ifndef SIMULATION_H
#define SIMULATION_H

#include "box.h"
#include "triangle.h"
#include "camera.h"

class Simulation {
    // Window parameters
    int width;
    int height;

    // Simulation state
    Box b;
    Triangle t;
    PointMass p;
    PointMass v[3];
    Camera c;
    PointMass* closest;
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
