#ifndef SPRING_H
#define SPRING_H

#include "interval.h"
#include "pointmass.h"

#include <vector>

class Spring {
    std::vector<PointMass>& mPoints;
    int mP;
    int mQ;
    float mK;
    float mL;

  public:
    Spring(std::vector<PointMass>& points, int p, int q, float length, float k);
    Spring(std::vector<PointMass>& points, int p, int q, float k);
    void draw();
    void contributeImpulse(float h);
};

#endif
