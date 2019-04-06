#include "edge.h"

#include <GLUT/glut.h>

#include "utils.h"

Edge::Edge(std::vector<PointMass>& p, int x0, int x1) : mP(p) {
    mV[0] = x0;
    mV[1] = x1;
}

void Edge::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glVertex3f(mP[mV[0]].mPos(0), mP[mV[0]].mPos(1), mP[mV[0]].mPos(2));
    glVertex3f(mP[mV[1]].mPos(0), mP[mV[1]].mPos(1), mP[mV[1]].mPos(2));
    glEnd();
}

void Edge::simulate(float h) {
    for(int i = 0; i < 2; i++) {
        mP[mV[i]].simulate(h);
    }
}

Eigen::Vector3f Edge::getPos(int i) {
    return mP[mV[i]].mPos;
}

Eigen::Vector3f Edge::getVel(int i) {
    return mP[mV[i]].mVel;
}

Eigen::Vector3f Edge::getFastest() {
    Eigen::Vector3f fastest = getVel(0);
    if(getVel(1).norm() > fastest.norm())
        fastest = getVel(1);
    return fastest;
}

Eigen::Vector3f Edge::getFurthest(Eigen::Vector3f center) {
    Eigen::Vector3f furthest = getPos(0);
    if((getPos(1) - center).norm() > (furthest - center).norm())
        furthest = getPos(1);
    return furthest;
}

Eigen::Vector3f Edge::getCentroid() {
    return (getPos(0) + getPos(1)) / 2.0f;
}

Eigen::Vector3f Edge::getNormal(Edge& e) {
    Eigen::Vector3f a = getPos(0) - getPos(1);
    Eigen::Vector3f b = e.getPos(0) - e.getPos(1);
    return a.cross(b);
}

bool Edge::isCoplanar(Edge other) {
    Eigen::Vector3f a, b, c, d;
    a = getPos(0);
    b = getPos(1);
    c = other.getPos(0);
    d = other.getPos(1);

    Eigen::Vector3f normal = (c - a).cross(c - a);
    Eigen::Vector3f someDir = d - a;

    float coplanarness = normal.dot(someDir);

    return approx(coplanarness, 0, 0.001);
}

Collision Edge::collide(Edge ed, float h) {
    // Make a temporary copy of the current state
    PointMass temp[4];
    temp[0] = PointMass(mP[mV[0]]);
    temp[1] = PointMass(mP[mV[1]]);
    temp[2] = PointMass(ed.mP[ed.mV[0]]);
    temp[3] = PointMass(ed.mP[ed.mV[1]]);

    std::vector<PointMass> tempPoint(std::begin(temp), std::end(temp));
    Edge e1(tempPoint, 0, 1);
    Edge e2(tempPoint, 2, 3);

    // Step back in time using the temporary copy
    e1.simulate(-1 * h);
    e2.simulate(-1 * h);

    // Calculate the (3) times that they will be coplanar
    // Compute the 4 coefficients of the cubic equation
    Eigen::Vector3f x21 = e1.getPos(1) - e1.getPos(0);
    Eigen::Vector3f x31 = e2.getPos(0) - e1.getPos(0);
    Eigen::Vector3f x41 = e2.getPos(1) - e1.getPos(0);

    Eigen::Vector3f v21 = e1.getVel(1) - e1.getVel(0);
    Eigen::Vector3f v31 = e2.getVel(0) - e1.getVel(0);
    Eigen::Vector3f v41 = e2.getVel(1) - e1.getVel(0);

    float a3 = v21.cross(v31).dot(v41);
    float a2 = v21.cross(v31).dot(x41) + (v21.cross(x31) + x21.cross(v31)).dot(v41);
    float a1 = (v21.cross(x31) + x21.cross(v31)).dot(x41) + x21.cross(x31).dot(v41);
    float a0 = x21.cross(x31).dot(x41);

    float smallest = smallestPosRealRoot(a0, a1, a2, a3);

    if(smallest > h || smallest == -1) {
        return Collision();
    }

    // Advance in time up until the collision and then
    // check for line intersection
    e1.simulate(smallest);
    e2.simulate(smallest);
    Eigen::Vector2f coord = intersect(e1.getPos(0), e1.getPos(1), e2.getPos(0), e2.getPos(1));

    for(int i = 0; i < 2; i++) {
         if(coord[i] > 1 || coord[i] < 0 || isnan(coord[i])) {
            return Collision();
        }
    }

    return Collision(Collision::EDGEEDGE, -1, -1, smallest, coord, Eigen::Vector3f());
}

PointMass Edge::interpolate(float t) {
    return mP[0] * (1.0f - t) + mP[1] * t;
}
