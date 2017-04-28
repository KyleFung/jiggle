#ifndef COLLISION_H
#define COLLISION_H

class Collision {
  public:
    enum ColType {EDGEEDGE, POINTFACE, FACEPOINT, NONE};
    Collision();
    Collision(ColType c, int A, int B, float t);
    bool exists();
    ColType getType();

    // Base indices of intersecting geometry and intersection time
    int indexA;
    int indexB;
    float t;

  private:
    ColType type;
    bool existence;
};

#endif
