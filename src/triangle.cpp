#include "triangle.h"

#include <GLUT/glut.h>

#include "utils.h"

Triangle::Triangle(PointMass* x0, PointMass* x1, PointMass* x2) {
    mV[0] = x0;
    mV[1] = x1;
    mV[2] = x2;
}

void Triangle::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    for(int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        glVertex3f(mV[i]->mPos(0), mV[i]->mPos(1), mV[i]->mPos(2));
        glVertex3f(mV[j]->mPos(0), mV[j]->mPos(1), mV[j]->mPos(2));
    }
    glEnd();
}

void Triangle::simulate(float h) {
    for(int i = 0; i < 3; i++) {
        mV[i]->simulate(h);
    }
}

float Triangle::collide(PointMass p, float h) {
    // Calculate the previous state by stepping back in time
    PointMass tempTriangle[3];
    for(int i = 0; i < 3; i++) {
        tempTriangle[i] = PointMass(*mV[i]);
    }
    Triangle t0(tempTriangle, tempTriangle + 1, tempTriangle + 2);
    PointMass p0(p);
    t0.simulate(-1 * h);
    p0.simulate(-1 * h);

    // Calculate the (3) times that they will be coplanar
    // Compute the 4 coefficients of the cubic equation
    Eigen::Vector3f a = t0.mV[1]->mPos - t0.mV[0]->mPos;
    Eigen::Vector3f b = t0.mV[1]->mVel - t0.mV[0]->mVel;
    Eigen::Vector3f c = t0.mV[2]->mPos - t0.mV[0]->mPos;
    Eigen::Vector3f d = t0.mV[2]->mVel - t0.mV[0]->mVel;
    Eigen::Vector3f e = p0.mPos - t0.mV[0]->mPos;
    Eigen::Vector3f f = p0.mVel - t0.mV[0]->mVel;

    Eigen::Vector3f bxc = b.cross(c);
    Eigen::Vector3f bxd = b.cross(d);
    Eigen::Vector3f axc = a.cross(c);
    Eigen::Vector3f axd = a.cross(d);

    float a0 = e.dot(axc);
    float a1 = f.dot(axc) + e.dot(bxc + axd);
    float a2 = e.dot(bxd) + f.dot(bxc + axd);
    float a3 = f.dot(bxd);

    float smallest = smallestPosRealRoot(a0, a1, a2, a3);

    if(smallest > h || smallest == -1) {
        return -1;
    }

    // Advance in time up until the collision and then
    // check the barycentric coordinates
    t0.simulate(smallest);
    p0.simulate(smallest);
    Eigen::Vector3f coord = bary(t0.mV[0]->mPos, t0.mV[1]->mPos, t0.mV[2]->mPos,
                                 p0.mPos);
    for(int i = 0; i < 3; i++) {
        if(coord[i] > 1 || coord[i] < 0) {
            return -1;
        }
    }

    return smallest;
}
