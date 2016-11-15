#include "triangle.h"
Triangle::Triangle(PointMass* x0, PointMass* x1, PointMass* x2) {
    mV[0] = x0;
    mV[1] = x1;
    mV[2] = x2;
    mS[0] = Spring(x0, x1, 1);
    mS[1] = Spring(x1, x2, 1);
    mS[2] = Spring(x2, x0, 1);
}

void Triangle::draw() {
    for(int i = 0; i < 3; i++) {
        mS[i].draw();
    }
}

void Triangle::simulate(float h) {
    // Assume springs have no internal force for now
    for(int i = 0; i < 3; i++) {
        mV[i]->simulate(h);
    }
}
