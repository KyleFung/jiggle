#include "spring.h"

#include <GLUT/glut.h>

Spring::Spring(interval<PointMass>& points, int p, int q, float length, float k) : mPoints(points) {
    mP = p;
    mQ = q;
    mL = length;
    mK = k;
}

Spring::Spring(interval<PointMass>& points, int p, int q, float k) : mPoints(points) {
    mP = p;
    mQ = q;
    mL = (mPoints[mP].mPos - mPoints[mQ].mPos).norm();
    mK = k;
}

void Spring::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glVertex3f(mPoints[mP].mPos(0), mPoints[mP].mPos(1), mPoints[mP].mPos(2));
    glVertex3f(mPoints[mQ].mPos(0), mPoints[mQ].mPos(1), mPoints[mQ].mPos(2));
    glEnd();
}

void Spring::contributeImpulse(float h) {
    // Calculate hooke force
    Eigen::Vector3f diff = (mPoints[mP].mPos - mPoints[mQ].mPos);
    float distance = diff.norm();

    // Record force into point
    Eigen::Vector3f qFrc = ((mK / distance) * diff) * (distance - mL);
    Eigen::Vector3f pFrc = -1 * qFrc;

    // Account for drag
    qFrc -= 0.01 * mPoints[mQ].mVel;
    pFrc -= 0.01 * mPoints[mP].mVel;

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
    v0 << mPoints[mP].mVel, mPoints[mQ].mVel;

    Eigen::MatrixXf b(6, 6);
    b = h * (f0 + h * dfdx * v0);

    // Solve
    dv = A.colPivHouseholderQr().solve(b);

    // Integrate velocity
    mPoints[mP].mVel += dv.head(3);
    mPoints[mQ].mVel += dv.tail(3);
}
