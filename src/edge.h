#ifndef EDGE_H
#define EDGE_H

#include "pointmass.h"

class Edge {
  public:
    PointMass* mV[2];
    Edge(PointMass* x0, PointMass* x1);
    void draw();
    void simulate(float h);
    float collide(Edge e, float h);
};

#endif
