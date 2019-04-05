#include "collision.h"
#include <algorithm>

Collision::Collision() : type(NONE), existence(false), t(-1), indexA(-1), indexB(-1) {}

Collision::Collision(Collision::ColType c, int A, int B, float time, Eigen::Vector2f pos, Eigen::Vector3f normal) {
    Collision();
    type = c;
    t = time;
    pos = pos;
    nor = normal;
    if(type != NONE) {
        existence = true;
        indexA = A;
        indexB = B;
    }
}

void Collision::flip() {
    int A = indexA;
    indexA = indexB;
    indexB = A;
    if(type == Collision::NONE || type == Collision::EDGEEDGE) {
        std::swap(pos.x(), pos.y());
        return;
    }
    else if(type == Collision::POINTFACE) {
        type = Collision::FACEPOINT;
    }
    else {
        type = Collision::POINTFACE;
    }
}

bool Collision::exists() {
    return existence;
}

Collision::ColType Collision::getType() {
    return type;
}
