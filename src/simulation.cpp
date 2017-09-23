#include "simulation.h"

#include "pointmass.h"

#include <Eigen/Geometry>
#include <GLUT/glut.h>
#include <iostream>

Simulation::Simulation() : width(500), height(500),
                           c(Eigen::Vector3f(0, 5, 50), 0, 0, width, height, 20),
                           closest(NULL), chosenDist(0), mode(0),
                           frozen(false), gravity(false), gForce(10) {
    b[0].setName("Lower box");
    b[1].setName("Upper box");
    b[0].translate(Eigen::Vector3f(0, 0, 0));
    b[1].rotate(Eigen::Vector3f(0, 1, 1), 90);
    b[1].translate(Eigen::Vector3f(0, 5, 1));
}

void Simulation::start() {
    // Set up projection matrix for first time
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(c.mFovy, 1, 1, 1000);
    glViewport(0, 0, 500, 500);
    glMatrixMode(GL_MODELVIEW);

    warpMouse(width / 2, height / 2);
    glutMainLoop();
}

void Simulation::simulate() {
    if(frozen) {
        return;
    }

    // Step forward
    float h = 0.03;
    for(int i = 0; i < numBoxes; i++) {
        if(gravity) {
            b[i].applyGravity(gForce, h);
        }
        b[i].simulate(h);

        // Do collision detection with floor
        b[i].collideFloor(-10);
    }

    // Do collision checking. If collision, then freeze the scene
    for(int i = 0; i < numBoxes; i++) {
        for(int j = i + 1; j < numBoxes; j++) {
            // We fix up to a certain number of collisions
            for(int k = 0; k < 10; k++) {
                Collision c = b[i].collide(b[j], h);
                if(c.exists()) {
                    // Correct the intersection
                    float eps = 0.07 * h;
                    float reverseStep = c.t - h - eps;
                    if(c.getType() == Collision::EDGEEDGE) {
                        b[i].getGeometry().getEdges().getItemFromBase(c.indexA).simulate(reverseStep);
                        b[j].getGeometry().getEdges().getItemFromBase(c.indexB).simulate(reverseStep);
                    }
                    else if(c.getType() == Collision::FACEPOINT) {
                        b[i].getGeometry().getTriangles().getItemFromBase(c.indexA).simulate(reverseStep);
                        b[j].getGeometry().getPoints().getItemFromBase(c.indexB).simulate(reverseStep);
                    }
                    else if(c.getType() == Collision::POINTFACE) {
                        b[i].getGeometry().getPoints().getItemFromBase(c.indexA).simulate(reverseStep);
                        b[j].getGeometry().getTriangles().getItemFromBase(c.indexB).simulate(reverseStep);
                    }
                    frozen = true;
                }
                else {
                    break;
                }
            }
        }
    }
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
    for(int i = 0; i < numBoxes; i++) {
        b[i].draw();
    }

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
        warpMouse(width / 2, height / 2);
    }
    // Apply gravity
    if(key == 'g') {
        gravity = !gravity;
    }
    // Freeze/unfreeze frame
    if(key == 'f') {
        frozen = !frozen;
    }
    // Print camera info
    if(key == 'c') {
        std::cout << "Pos: " << c.mPos.transpose() << "\n";
        std::cout << "Pitch: " << c.mPitch << "\n";
        std::cout << "Yaw: " << c.mYaw << "\n";
    }

    c.onKey(key);
}

void Simulation::mousePassiveHandler(int x, int y) {
    // Only move around camera if not in picking mode
    if(mode == 0) {
        c.onMouse(x, y);
        warpMouse(width / 2, height / 2);
    }
}

void Simulation::mousePressedHandler(int x, int y) {
    // Move camera around if not in picking mode
    if(mode == 0) {
        c.onMouse(x, y);
        warpMouse(width / 2, height / 2);
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
            closest = getClosestPoint(0.5, c.mPos, ray);
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

void Simulation::warpMouse(int x, int y) {
#ifdef _WIN32
    // Windows GLUT seems to trigger another mouse move when we warp
    static int count = 0;
    count = (count + 1) % 2;
    if (count > 0) {
        return;
    }
#endif
    glutWarpPointer(width / 2, height / 2);
}

PointMass* Simulation::getClosestPoint(float tol, Eigen::Vector3f pos, Eigen::Vector3f ray) {
    PointMass* closest = NULL;
    float depth = 10000;
    for(int i = 0; i < numBoxes; i++) {
        PointMass* current = b[i].getClosestPoint(tol, pos, ray);
        float currentDepth = current == NULL ? 10000 : current->calculateDepth(tol, pos, ray);

        if(current != NULL && currentDepth < depth) {
            closest = current;
            depth = currentDepth;
        }
    }
    return closest;
}
