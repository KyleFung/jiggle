#include "collision.h"

Collision::Collision() : type(NONE), existence(false), t(-1), mEdgeA(-1), mEdgeB(-1), mFace(-1), mPoint(-1) {}

Collision::Collision(Collision::ColType c, int A, int B, float time) {
    Collision();
    type = c;
    t = time;
    if(c == Collision::EDGEEDGE) {
        existence = true;
        mEdgeA = A;
        mEdgeB = B;
    }
    else if(c == Collision::POINTFACE) {
        existence = true;
        mPoint = A;
        mFace = B;
    }
}

bool Collision::exists() {
    return existence;
}