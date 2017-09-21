#include "pointmass.h"

#include <GLUT/glut.h>

PointMass::PointMass() {
    mPos << 0, 0, 0;
    mVel << 0, 0, 0;
    mMass = 1;
    mImmobile = false;
    mID = -1;
}

PointMass::PointMass(const PointMass& p) {
    mPos = p.mPos;
    mVel = p.mVel;
    mMass = p.mMass;
    mImmobile = p.mImmobile;
    mID = p.mID;
}

PointMass::PointMass(float x, float y, float z, int i) {
    mPos << x, y, z;
    mVel << 0, 0, 0;
    mMass = 1;
    mImmobile = false;
    mID = i;
}

void PointMass::simulate(float h) {
    if(mImmobile) {
        mVel << 0, 0, 0;
        return;
    }

    mPos += h * mVel;
}

float PointMass::calculateDepth(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir) {
    Eigen::Vector3f pos = mPos - p;

    float depth = dir.dot(pos);
    Eigen::Vector3f projection = depth * dir;

    Eigen::Vector3f skew = pos - projection;
    float distanceSquared = skew.dot(skew);

    // If the point is too far from the ray, we just say its behind the camera
    if(distanceSquared > minDist * minDist) {
        return -1;
    }
    return depth;
}

void PointMass::draw() {
    glTranslatef(mPos(0), mPos(1), mPos(2));
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glutWireSphere(0.2, 10, 10);
    glTranslatef(-1 * mPos(0), -1 * mPos(1), -1 * mPos(2));
}
