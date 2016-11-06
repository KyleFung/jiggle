#include "box.h"

#include <GLUT/glut.h>
#include <math.h>

Box::Box() {
    mK = 300;

    mP[0] = PointMass(-1, +1, +1);
    mP[1] = PointMass(-1, +1, -1);
    mP[2] = PointMass(+1, +1, -1);
    mP[3] = PointMass(+1, +1, +1);
    mP[4] = PointMass(-1, -1, +1);
    mP[5] = PointMass(-1, -1, -1);
    mP[6] = PointMass(+1, -1, -1);
    mP[7] = PointMass(+1, -1, +1);

    mS[0] = Spring(&mP[0], &mP[1], 2, mK);
    mS[1] = Spring(&mP[1], &mP[2], 2, mK);
    mS[2] = Spring(&mP[2], &mP[3], 2, mK);
    mS[3] = Spring(&mP[3], &mP[0], 2, mK);
    mS[4] = Spring(&mP[0], &mP[4], 2, mK);
    mS[5] = Spring(&mP[1], &mP[5], 2, mK);
    mS[6] = Spring(&mP[2], &mP[6], 2, mK);
    mS[7] = Spring(&mP[3], &mP[7], 2, mK);
    mS[8] = Spring(&mP[4], &mP[5], 2, mK);
    mS[9] = Spring(&mP[5], &mP[6], 2, mK);
    mS[10] = Spring(&mP[6], &mP[7], 2, mK);
    mS[11] = Spring(&mP[7], &mP[4], 2, mK);

    // Cross springs
    mS[12] = Spring(&mP[0], &mP[6], std::sqrt(12), mK);
    mS[13] = Spring(&mP[1], &mP[7], std::sqrt(12), mK);
    mS[14] = Spring(&mP[2], &mP[4], std::sqrt(12), mK);
    mS[15] = Spring(&mP[3], &mP[5], std::sqrt(12), mK);

    // Face springs
    mS[16] = Spring(&mP[0], &mP[2], std::sqrt(8), mK);
    mS[17] = Spring(&mP[1], &mP[3], std::sqrt(8), mK);
    mS[18] = Spring(&mP[0], &mP[5], std::sqrt(8), mK);
    mS[19] = Spring(&mP[1], &mP[4], std::sqrt(8), mK);
    mS[20] = Spring(&mP[4], &mP[6], std::sqrt(8), mK);
    mS[21] = Spring(&mP[5], &mP[7], std::sqrt(8), mK);
    mS[22] = Spring(&mP[2], &mP[7], std::sqrt(8), mK);
    mS[23] = Spring(&mP[3], &mP[6], std::sqrt(8), mK);
    mS[24] = Spring(&mP[0], &mP[7], std::sqrt(8), mK);
    mS[25] = Spring(&mP[3], &mP[4], std::sqrt(8), mK);
    mS[26] = Spring(&mP[1], &mP[6], std::sqrt(8), mK);
    mS[27] = Spring(&mP[2], &mP[5], std::sqrt(8), mK);
}

void Box::draw() {
    // Draw faces
    glBegin(GL_QUADS);
    glColor4f(0.7, 0.0, 0.0, 1.0);
    glVertex3f(mP[0].mPos(0), mP[0].mPos(1), mP[0].mPos(2));
    glVertex3f(mP[1].mPos(0), mP[1].mPos(1), mP[1].mPos(2));
    glVertex3f(mP[2].mPos(0), mP[2].mPos(1), mP[2].mPos(2));
    glVertex3f(mP[3].mPos(0), mP[3].mPos(1), mP[3].mPos(2));

    glColor4f(0.0, 0.7, 0.0, 1.0);
    glVertex3f(mP[0].mPos(0), mP[0].mPos(1), mP[0].mPos(2));
    glVertex3f(mP[1].mPos(0), mP[1].mPos(1), mP[1].mPos(2));
    glVertex3f(mP[5].mPos(0), mP[5].mPos(1), mP[5].mPos(2));
    glVertex3f(mP[4].mPos(0), mP[4].mPos(1), mP[4].mPos(2));

    glColor4f(0.0, 0.0, 0.7, 1.0);
    glVertex3f(mP[4].mPos(0), mP[4].mPos(1), mP[4].mPos(2));
    glVertex3f(mP[5].mPos(0), mP[5].mPos(1), mP[5].mPos(2));
    glVertex3f(mP[6].mPos(0), mP[6].mPos(1), mP[6].mPos(2));
    glVertex3f(mP[7].mPos(0), mP[7].mPos(1), mP[7].mPos(2));

    glColor4f(0.0, 0.7, 0.7, 1.0);
    glVertex3f(mP[2].mPos(0), mP[2].mPos(1), mP[2].mPos(2));
    glVertex3f(mP[3].mPos(0), mP[3].mPos(1), mP[3].mPos(2));
    glVertex3f(mP[7].mPos(0), mP[7].mPos(1), mP[7].mPos(2));
    glVertex3f(mP[6].mPos(0), mP[6].mPos(1), mP[6].mPos(2));

    glColor4f(0.7, 0.0, 0.7, 1.0);
    glVertex3f(mP[0].mPos(0), mP[0].mPos(1), mP[0].mPos(2));
    glVertex3f(mP[3].mPos(0), mP[3].mPos(1), mP[3].mPos(2));
    glVertex3f(mP[7].mPos(0), mP[7].mPos(1), mP[7].mPos(2));
    glVertex3f(mP[4].mPos(0), mP[4].mPos(1), mP[4].mPos(2));

    glColor4f(0.7, 0.7, 0.0, 1.0);
    glVertex3f(mP[1].mPos(0), mP[1].mPos(1), mP[1].mPos(2));
    glVertex3f(mP[2].mPos(0), mP[2].mPos(1), mP[2].mPos(2));
    glVertex3f(mP[6].mPos(0), mP[6].mPos(1), mP[6].mPos(2));
    glVertex3f(mP[5].mPos(0), mP[5].mPos(1), mP[5].mPos(2));

    glColor4f(0.0, 0.0, 0.0, 1.0);

    glEnd();
}

void Box::applyGravity(float g, float h) {
    for(int i = 0; i < 8; i++) {
        mP[i].mVel(1) -= (g * h) / mP[i].mMass;
    }
}

void Box::collideFloor(float level) {
    for(int i = 0; i < 8; i++) {
        if(mP[i].mPos(1) <= level) {
            mP[i].mPos(1) = level + 0.001;
            mP[i].mVel(1) = std::abs(mP[i].mVel(1));
            mP[i].mVel *= 0.9;
        }
    }
}

void Box::simulate(float h) {
    for(int i = 0; i < mNumSprings; i++) {
        mS[i].contributeImpulse(h);
    }
    for(int i = 0; i < 8; i++) {
        mP[i].simulate(h);
    }
}

PointMass* Box::getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir) {
    PointMass* closest = NULL;
    float closestDepth = 10000;
    for(int i = 0; i < 8; i++) {
        float depth = mP[i].calculateDepth(minDist, p, dir);
        // If the point is not behind the camera, and closer than the previous point
        if(depth > 0 && depth < closestDepth) {
            closest = &mP[i];
            closestDepth = depth;
        }
    }
    return closest;
}
