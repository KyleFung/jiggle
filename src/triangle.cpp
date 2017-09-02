#include "triangle.h"

#include <GLUT/glut.h>

#include <iterator>

#include "utils.h"

Triangle::Triangle(std::vector<PointMass>& p, int x0, int x1, int x2) : mP(p) {
    mV[0] = x0;
    mV[1] = x1;
    mV[2] = x2;
}

void Triangle::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    for(int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        glVertex3f(mP[mV[i]].mPos(0), mP[mV[i]].mPos(1), mP[mV[i]].mPos(2));
        glVertex3f(mP[mV[j]].mPos(0), mP[mV[j]].mPos(1), mP[mV[j]].mPos(2));
    }
    glEnd();
}

void Triangle::simulate(float h) {
    for(int i = 0; i < 3; i++) {
        mP[mV[i]].simulate(h);
    }
}

Eigen::Vector3f Triangle::getPos(int i) {
    return mP[mV[i]].mPos;
}

Eigen::Vector3f Triangle::getVel(int i) {
    return mP[mV[i]].mVel;
}

Eigen::Vector3f Triangle::getFastest() {
    Eigen::Vector3f fastest = getVel(0);
    if(getVel(1).norm() > fastest.norm())
        fastest = getVel(1);
    if(getVel(2).norm() > fastest.norm())
        fastest = getVel(2);
    return fastest;
}

Eigen::Vector3f Triangle::getFurthest(Eigen::Vector3f center) {
    Eigen::Vector3f furthest = getPos(0);
    if((getPos(1) - center).norm() > (furthest - center).norm())
        furthest = getPos(1);
    if((getPos(2) - center).norm() > (furthest - center).norm())
        furthest = getPos(2);
    return furthest;
}

Eigen::Vector3f Triangle::getCentroid() {
    return (getPos(0) + getPos(1) + getPos(2)) / 3.0f;
}

Eigen::Vector3f Triangle::getNormal() {
    Eigen::Vector3f a, b, c;
    a = getPos(0);
    b = getPos(1);
    c = getPos(2);

    Eigen::Vector3f normal = (c - a).cross(b - a);
    return normal.normalized();
}

bool Triangle::isCoplanar(PointMass p) {
    Eigen::Vector3f a, b, c, d;
    a = getPos(0);
    b = getPos(1);
    c = getPos(2);
    d = p.mPos;

    Eigen::Vector3f normal = (c - a).cross(b - a);
    Eigen::Vector3f someDir = d - a;

    float coplanarness = normal.dot(someDir);

    return approx(coplanarness, 0, 0.0000001);
}

float Triangle::collide(PointMass p, float h) {
    // Calculate the previous state by stepping back in time
    PointMass tempTriangle[3];
    int tempIndex[3];
    for(int i = 0; i < 3; i++) {
        tempTriangle[i] = PointMass(mP[mV[i]]);
        tempIndex[i] = i;
    }

    std::vector<PointMass> tempPoint(std::begin(tempTriangle), std::end(tempTriangle));
    Triangle t0(tempPoint, 0, 1, 2);
    PointMass p0(p);
    t0.simulate(-1 * h);
    p0.simulate(-1 * h);

    // Calculate the (3) times that they will be coplanar
    // Compute the 4 coefficients of the cubic equation
    Eigen::Vector3f a = t0.getPos(1) - t0.getPos(0);
    Eigen::Vector3f b = t0.getVel(1) - t0.getVel(0);
    Eigen::Vector3f c = t0.getPos(2) - t0.getPos(0);
    Eigen::Vector3f d = t0.getVel(2) - t0.getVel(0);
    Eigen::Vector3f e = p0.mPos - t0.getPos(0);
    Eigen::Vector3f f = p0.mPos - t0.getVel(0);

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
    Eigen::Vector3f coord = bary(t0.getPos(0), t0.getPos(1), t0.getPos(2), p0.mPos);
    for(int i = 0; i < 3; i++) {
         if(coord[i] > 1 || coord[i] < 0 || isnan(coord[i])) {
            return -1;
        }
    }

    return smallest;
}
