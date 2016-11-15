#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "pointmass.h"
#include "spring.h"

class Triangle {
  public:
    PointMass* mV[3];
    Spring mS[3];
    Triangle(PointMass* x0, PointMass* x1, PointMass* x2);
    void draw();
    void simulate(float h);
};

#endif
