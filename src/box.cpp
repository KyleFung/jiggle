#include "box.h"

#include <GLUT/glut.h>

Box::Box(Eigen::Vector3f pos) {
    init();
    translate(pos);
    refreshBounding(0);
}

Box::Box() {
    init();
    refreshBounding(0);
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

void Box::init() {
    mK = 300;

    // Add the vertices that make up a box
    // Add corners
    addPoint(PointMass(-1, +1, +1));
    addPoint(PointMass(-1, +1, -1));
    addPoint(PointMass(+1, +1, -1));
    addPoint(PointMass(+1, +1, +1));
    addPoint(PointMass(-1, -1, +1));
    addPoint(PointMass(-1, -1, -1));
    addPoint(PointMass(+1, -1, -1));
    addPoint(PointMass(+1, -1, +1));
    // Add face centers
    addPoint(PointMass(1, 0, 0));
    addPoint(PointMass(0, 1, 0));
    addPoint(PointMass(0, 0, 1));
    addPoint(PointMass(-1, 0, 0));
    addPoint(PointMass(0, -1, 0));
    addPoint(PointMass(0, 0, -1));

    // Add the springs that make up the box
    // Outside edges
    addSpring(Spring(getPoint(0), getPoint(1), mK));
    addSpring(Spring(getPoint(1), getPoint(2), mK));
    addSpring(Spring(getPoint(2), getPoint(3), mK));
    addSpring(Spring(getPoint(3), getPoint(0), mK));
    addSpring(Spring(getPoint(0), getPoint(4), mK));
    addSpring(Spring(getPoint(1), getPoint(5), mK));
    addSpring(Spring(getPoint(2), getPoint(6), mK));
    addSpring(Spring(getPoint(3), getPoint(7), mK));
    addSpring(Spring(getPoint(4), getPoint(5), mK));
    addSpring(Spring(getPoint(5), getPoint(6), mK));
    addSpring(Spring(getPoint(6), getPoint(7), mK));
    addSpring(Spring(getPoint(7), getPoint(4), mK));
    // Face springs
    // Top
    addSpring(Spring(getPoint(0), getPoint(9), mK));
    addSpring(Spring(getPoint(1), getPoint(9), mK));
    addSpring(Spring(getPoint(2), getPoint(9), mK));
    addSpring(Spring(getPoint(3), getPoint(9), mK));
    // Right
    addSpring(Spring(getPoint(2), getPoint(8), mK));
    addSpring(Spring(getPoint(3), getPoint(8), mK));
    addSpring(Spring(getPoint(6), getPoint(8), mK));
    addSpring(Spring(getPoint(7), getPoint(8), mK));
    // Bottom
    addSpring(Spring(getPoint(4), getPoint(12), mK));
    addSpring(Spring(getPoint(5), getPoint(12), mK));
    addSpring(Spring(getPoint(6), getPoint(12), mK));
    addSpring(Spring(getPoint(7), getPoint(12), mK));
    // Left
    addSpring(Spring(getPoint(0), getPoint(11), mK));
    addSpring(Spring(getPoint(1), getPoint(11), mK));
    addSpring(Spring(getPoint(4), getPoint(11), mK));
    addSpring(Spring(getPoint(5), getPoint(11), mK));
    // Front
    addSpring(Spring(getPoint(0), getPoint(10), mK));
    addSpring(Spring(getPoint(3), getPoint(10), mK));
    addSpring(Spring(getPoint(4), getPoint(10), mK));
    addSpring(Spring(getPoint(7), getPoint(10), mK));
    // Back
    addSpring(Spring(getPoint(1), getPoint(13), mK));
    addSpring(Spring(getPoint(2), getPoint(13), mK));
    addSpring(Spring(getPoint(5), getPoint(13), mK));
    addSpring(Spring(getPoint(6), getPoint(13), mK));
    // Cross Springs
    addSpring(Spring(getPoint(0), getPoint(6), mK));
    addSpring(Spring(getPoint(1), getPoint(7), mK));
    addSpring(Spring(getPoint(2), getPoint(4), mK));
    addSpring(Spring(getPoint(3), getPoint(5), mK));
    addSpring(Spring(getPoint(8), getPoint(11), mK));
    addSpring(Spring(getPoint(9), getPoint(12), mK));
    addSpring(Spring(getPoint(10), getPoint(13), mK));
    addSpring(Spring(getPoint(8), getPoint(9), mK));
    addSpring(Spring(getPoint(9), getPoint(11), mK));
    addSpring(Spring(getPoint(11), getPoint(12), mK));
    addSpring(Spring(getPoint(12), getPoint(8), mK));
    addSpring(Spring(getPoint(9), getPoint(10), mK));
    addSpring(Spring(getPoint(10), getPoint(12), mK));
    addSpring(Spring(getPoint(12), getPoint(13), mK));
    addSpring(Spring(getPoint(13), getPoint(9), mK));
    addSpring(Spring(getPoint(8), getPoint(13), mK));
    addSpring(Spring(getPoint(13), getPoint(11), mK));
    addSpring(Spring(getPoint(11), getPoint(10), mK));
    addSpring(Spring(getPoint(10), getPoint(8), mK));

    // Create the faces
    // Top
    addTriangle(Triangle(getPointList(), 1, 0, 9));
    addTriangle(Triangle(getPointList(), 2, 1, 9));
    addTriangle(Triangle(getPointList(), 3, 2, 9));
    addTriangle(Triangle(getPointList(), 0, 3, 9));
    // Right
    addTriangle(Triangle(getPointList(), 2, 3, 8));
    addTriangle(Triangle(getPointList(), 6, 2, 8));
    addTriangle(Triangle(getPointList(), 7, 6, 8));
    addTriangle(Triangle(getPointList(), 3, 7, 8));
    // Bottom
    addTriangle(Triangle(getPointList(), 5, 6, 12));
    addTriangle(Triangle(getPointList(), 6, 7, 12));
    addTriangle(Triangle(getPointList(), 7, 4, 12));
    addTriangle(Triangle(getPointList(), 4, 5, 12));
    // Left
    addTriangle(Triangle(getPointList(), 0, 1, 11));
    addTriangle(Triangle(getPointList(), 4, 0, 11));
    addTriangle(Triangle(getPointList(), 5, 4, 11));
    addTriangle(Triangle(getPointList(), 1, 5, 11));
    // Front
    addTriangle(Triangle(getPointList(), 3, 0, 10));
    addTriangle(Triangle(getPointList(), 7, 3, 10));
    addTriangle(Triangle(getPointList(), 4, 7, 10));
    addTriangle(Triangle(getPointList(), 0, 4, 10));
    // Back
    addTriangle(Triangle(getPointList(), 1, 2, 13));
    addTriangle(Triangle(getPointList(), 5, 1, 13));
    addTriangle(Triangle(getPointList(), 6, 5, 13));
    addTriangle(Triangle(getPointList(), 2, 6, 13));

    // Add Edges
    addEdge(Edge(getPoint(0), getPoint(1)));
    addEdge(Edge(getPoint(1), getPoint(2)));
    addEdge(Edge(getPoint(2), getPoint(3)));
    addEdge(Edge(getPoint(3), getPoint(0)));
    addEdge(Edge(getPoint(0), getPoint(4)));
    addEdge(Edge(getPoint(1), getPoint(5)));
    addEdge(Edge(getPoint(2), getPoint(6)));
    addEdge(Edge(getPoint(3), getPoint(7)));
    addEdge(Edge(getPoint(4), getPoint(5)));
    addEdge(Edge(getPoint(5), getPoint(6)));
    addEdge(Edge(getPoint(6), getPoint(7)));
    addEdge(Edge(getPoint(7), getPoint(4)));
    // Face springs
    // Top
    addEdge(Edge(getPoint(0), getPoint(9)));
    addEdge(Edge(getPoint(1), getPoint(9)));
    addEdge(Edge(getPoint(2), getPoint(9)));
    addEdge(Edge(getPoint(3), getPoint(9)));
    // Right
    addEdge(Edge(getPoint(2), getPoint(8)));
    addEdge(Edge(getPoint(3), getPoint(8)));
    addEdge(Edge(getPoint(6), getPoint(8)));
    addEdge(Edge(getPoint(7), getPoint(8)));
    // Bottom
    addEdge(Edge(getPoint(4), getPoint(12)));
    addEdge(Edge(getPoint(5), getPoint(12)));
    addEdge(Edge(getPoint(6), getPoint(12)));
    addEdge(Edge(getPoint(7), getPoint(12)));
    // Left
    addEdge(Edge(getPoint(0), getPoint(11)));
    addEdge(Edge(getPoint(1), getPoint(11)));
    addEdge(Edge(getPoint(4), getPoint(11)));
    addEdge(Edge(getPoint(5), getPoint(11)));
    // Front
    addEdge(Edge(getPoint(0), getPoint(10)));
    addEdge(Edge(getPoint(3), getPoint(10)));
    addEdge(Edge(getPoint(4), getPoint(10)));
    addEdge(Edge(getPoint(7), getPoint(10)));
    // Back
    addEdge(Edge(getPoint(1), getPoint(13)));
    addEdge(Edge(getPoint(2), getPoint(13)));
    addEdge(Edge(getPoint(5), getPoint(13)));
    addEdge(Edge(getPoint(6), getPoint(13)));
}
