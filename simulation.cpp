#include "simulation.h"

#include "pointmass.h"

#include <Eigen/Geometry>
#include <GLUT/glut.h>

Simulation::Simulation() : t(p, p + 1, p + 2), width(500), height(500), c(500, 500, 20),
                           closest(NULL), chosenDist(0), mode(0),
                           gravity(false), gForce(10) {
    p[0].mPos << 0, 0, 0;
    p[1].mPos << 0, 5, 0;
    p[2].mPos << 0, 0, 5;
}

void Simulation::start() {
    // Set up projection matrix for first time
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(c.mFovy, 1, 1, 1000);
    glViewport(0, 0, 500, 500);
    glMatrixMode(GL_MODELVIEW);

    glutWarpPointer(width / 2, height / 2);
    glutMainLoop();
}

void Simulation::simulate() {
    float h = 0.03;
    // Step forward
    if(gravity) {
        //b.applyGravity(gForce, h);
    }
    //b.simulate(h);
    t.simulate(h);

    // Do collision detection with floor
    //b.collideFloor(-10);
}

void Simulation::displayFunc() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    Eigen::Vector3f up = c.getUp();
    Eigen::Vector3f dir = c.getDir();
    Eigen::Vector3f cen = c.mPos + dir;
    gluLookAt(c.mPos(0), c.mPos(1), c.mPos(2), cen(0), cen(1), cen(2), up(0), up(1), up(2));

    if(chosen) {
        closest->draw();
    }

    simulate();

    // Draw scene (just box and floor right now)
    //b.draw();
    t.draw();

    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glColor4f(0.7, 0.7, 0.7, 1.0);
    glVertex3f(100, -10, 100);
    glVertex3f(100, -10, -100);

    glColor4f(0.0, 0.0, 0.0, 1.0);
    glVertex3f(-100, -10, -100);
    glVertex3f(-100, -10, 100);
    glEnd();

    glutSwapBuffers();
}

void Simulation::keyHandler(unsigned char key, int x, int y) {
    if (key == 'q')
        exit(0);
    // Toggle picking mode
    if(key == 'e') {
        mode = (mode + 1) % 2;
        glutWarpPointer(width / 2, height / 2);
    }
    // Apply gravity
    if(key == 'g') {
        gravity = !gravity;
    }

    c.onKey(key);
}

void Simulation::mousePassiveHandler(int x, int y) {
    // Only move around camera if not in picking mode
    if(mode == 0) {
        c.onMouse(x, y);
        glutWarpPointer(width / 2, height / 2);
    }
}

void Simulation::mousePressedHandler(int x, int y) {
    // Move camera around if not in picking mode
    if(mode == 0) {
        c.onMouse(x, y);
        glutWarpPointer(width / 2, height / 2);
    }
    // Drag chosen object if in picking mode
    if(mode == 1 && chosen) {
        Eigen::Vector3f ray = c.unproject(x, y);
        closest->mPos = c.mPos + chosenDist * ray;

        // Simulate next step immediately
        simulate();
    }
}

void Simulation::mouseButtonHandler(int button, int state, int x, int y) {
    // If not in picking mode, do nothing 
    if(mode == 0) {
        return;
    }
    // If in picking mode, choose an object
    if(mode == 1) {
        // If we just pressed down, try to choose a point
        if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
            Eigen::Vector3f ray = c.unproject(x, y);
            closest = b.getClosestPoint(0.5, c.mPos, ray);

            chosen = closest == NULL ? false : true;
            if(closest != NULL) {
                chosenDist = closest->calculateDepth(0.5, c.mPos, ray);
                closest->mImmobile = true;
            }
        }
        // If we just let go, then unselect the point
        if(button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
            if(chosen && closest != NULL)
                closest->mImmobile = false;
            chosen = false;
            closest = NULL;
        }
    }
}
