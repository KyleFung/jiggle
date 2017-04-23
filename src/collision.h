#ifndef COLLISION_H
#define COLLISION_H

class Collision {
  public:
    enum ColType {EDGEEDGE, POINTFACE, NONE};
    Collision();
    Collision(ColType c, int A, int B, float t);
    bool exists();

  private:
    ColType type;
    bool existence;
    float t;

    // For intersections of Edge-Edge variety
    int mEdgeA;
    int mEdgeB;

    // For intersections of Face-Point variety
    int mFace;
    int mPoint;
};

#endif
