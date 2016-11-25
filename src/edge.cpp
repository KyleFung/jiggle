#include "edge.h"

#include <GLUT/glut.h>

#include "utils.h"

Edge::Edge(PointMass* x0, PointMass* x1) {
    mV[0] = x0;
    mV[1] = x1;
}

void Edge::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glVertex3f(mV[0]->mPos(0), mV[0]->mPos(1), mV[0]->mPos(2));
    glVertex3f(mV[1]->mPos(0), mV[1]->mPos(1), mV[1]->mPos(2));
    glEnd();
}

void Edge::simulate(float h) {
    for(int i = 0; i < 2; i++) {
        mV[i]->simulate(h);
    }
}

float Edge::collide(Edge ed, float h) {
    // Make a temporary copy of the current state
    PointMass temp1[2];
    PointMass temp2[2];
    temp1[0] = PointMass(*mV[0]);
    temp1[1] = PointMass(*mV[1]);
    temp2[0] = PointMass(*ed.mV[0]);
    temp2[1] = PointMass(*ed.mV[1]);

    Edge e1(temp1, temp1 + 1);
    Edge e2(temp2, temp2 + 1);

    // Step back in time using the temporary copy
    e1.simulate(-1 * h);
    e2.simulate(-1 * h);

    // Calculate the (3) times that they will be coplanar
    // Compute the 4 coefficients of the cubic equation
    Eigen::Vector3f a = e1.mV[0]->mPos - e1.mV[1]->mPos;
    Eigen::Vector3f b = e1.mV[0]->mVel - e1.mV[1]->mVel;
    Eigen::Vector3f c = e2.mV[0]->mPos - e2.mV[1]->mPos;
    Eigen::Vector3f d = e2.mV[0]->mVel - e2.mV[1]->mVel;
    Eigen::Vector3f e = e1.mV[0]->mPos - e2.mV[0]->mPos;
    Eigen::Vector3f f = e1.mV[0]->mVel - e2.mV[0]->mVel;

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
    // check for line intersection
    e1.simulate(smallest);
    e2.simulate(smallest);
    Eigen::Vector2f coord = intersect(e1.mV[0]->mPos, e1.mV[1]->mPos, e2.mV[0]->mPos, e2.mV[1]->mPos);

    for(int i = 0; i < 2; i++) {
         if(coord[i] > 1 || coord[i] < 0 || isnan(coord[i])) {
            return -1;
        }
    }

    return smallest;
}
