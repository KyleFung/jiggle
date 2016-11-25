#include "bounding.h"

Bounding::Bounding() {}

Bounding::Bounding(PointMass* p, int n, float h) {
    // If set of 0 points, then it is space of nothing
    if(n == 0) {
        mCen << 0, 0, 0;
        mRad = 0;
    }

    // Set cen to be the centroid of p
    mCen << 0, 0, 0;
    for(int i = 0; i < n; i++) {
        mCen += p[i].mPos;
    }
    mCen /= n;

    // Set rad to be the farthest distance from cen
    mRad = 0;
    for(int i = 0; i < n; i++) {
        float distance = (p[i].mPos - mCen).norm();
        if(distance > mRad)
            mRad = distance;
        distance = (p[i].mPos + h * p[i].mVel - mCen).norm();
        if(distance > mRad)
            mRad = distance;
    }
}

bool Bounding::collide(Bounding b) {
    float distance = (mCen - b.mCen).norm();
    if(distance > mRad + b.mRad) {
        return false;
    }
    return true;
}
