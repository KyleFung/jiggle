#include "collision.h"

Collision::Collision() : type(NONE), existence(false), t(-1), indexA(-1), indexB(-1) {}

Collision::Collision(Collision::ColType c, int A, int B, float time) {
    Collision();
    type = c;
    t = time;
    if(type != NONE) {
        existence = true;
        indexA = A;
        indexB = B;
    }
}

bool Collision::exists() {
    return existence;
}

Collision::ColType Collision::getType() {
    return type;
}