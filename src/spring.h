#ifndef SPRING_H
#define SPRING_H

class PointMass;

class Spring {
    PointMass* mP;
    PointMass* mQ;
    float mK;
    float mL;

  public:
    Spring() {}
    Spring(PointMass* p, PointMass* q, float length, float k);
    Spring(PointMass* p, PointMass* q, float k);
    void draw();
    void contributeImpulse(float h);
};

#endif
