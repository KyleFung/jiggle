#ifndef COLLISION_H
#define COLLISION_H

#include <Eigen/Dense>

class Collision {
  public:
    enum ColType {EDGEEDGE, POINTFACE, FACEPOINT, NONE};
    Collision();
    Collision(ColType c, int A, int B, float t, Eigen::Vector2f pos, Eigen::Vector3f nor);
    bool exists();
    ColType getType();

    // Reverses the roles of A and B
    void flip();

    // Base indices of intersecting geometry and intersection time
    int indexA;
    int indexB;
    float t;
    Eigen::Vector2f pos;
    Eigen::Vector3f nor;

  private:
    ColType type;
    bool existence;
};

#endif
