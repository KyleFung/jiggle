#include "spring.h"
#include "pointmass.h"

#include <GLUT/glut.h>
#include <math.h>

#include <iostream>

Spring::Spring(PointMass* p, PointMass* q, float length, float k) {
    mP = p;
    mQ = q;
    mL = length;
    mK = k;
}

void Spring::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glVertex3f(mP->mPos(0), mP->mPos(1), mP->mPos(2));
    glVertex3f(mQ->mPos(0), mQ->mPos(1), mQ->mPos(2));
    glEnd();
}

void Spring::contributeImpulse(float h) {
    // Calculate hooke force
    Eigen::Vector3f diff = (mP->mPos - mQ->mPos);
    float distance = diff.norm();

    // Record force into point
    Eigen::Vector3f qFrc = ((mK / distance) * diff) * (distance - mL);
    Eigen::Vector3f pFrc = -1 * qFrc;

    // Account for drag
    qFrc -= 0.01 * mQ->mVel;
    pFrc -= 0.01 * mP->mVel;

    Eigen::VectorXf dv(6);

    // Integrate for impulse
    // Explicit
    //dv << (pFrc * h) / mP->mMass, (qFrc * h) / mQ->mMass;

    // Implicit
    Eigen::MatrixXf dppE(3, 3);
    Eigen::Matrix3f I3 = Eigen::MatrixXf::Identity(3, 3);
    Eigen::MatrixXf outer = diff * diff.transpose();
    dppE = outer / (distance * distance);
    dppE += (I3 - (outer / (distance * distance))) * (1 - (mL / distance));
    dppE *= mK;

    Eigen::MatrixXf dfdx(6, 6);
    dfdx.block<3, 3>(0, 0) = -1 * dppE;
    dfdx.block<3, 3>(3, 3) = -1 * dppE;
    dfdx.block<3, 3>(0, 3) = dppE;
    dfdx.block<3, 3>(3, 0) = dppE;

    Eigen::MatrixXf dfdv(6, 6);
    Eigen::MatrixXf I6 = Eigen::MatrixXf::Identity(6, 6);
    dfdv = -0.01 * I6;

    Eigen::MatrixXf A(6, 6);
    A = (I6 - h * dfdv - h * h * dfdx);

    Eigen::VectorXf f0(6);
    Eigen::VectorXf v0(6);
    f0 << pFrc, qFrc;
    v0 << mP->mVel , mQ->mVel;

    Eigen::MatrixXf b(6, 6);
    b = h * (f0 + h * dfdx * v0);

    // Solve
    dv = A.colPivHouseholderQr().solve(b);

    // Integrate velocity
    mP->mVel += dv.head(3);
    mQ->mVel += dv.tail(3);
}
