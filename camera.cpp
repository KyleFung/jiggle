#include "camera.h"

#include <GLUT/glut.h>
#include "utils.h"

#include <iostream>

Camera::Camera(Eigen::Vector3f pos, float pitch, float yaw,
               int windowWidth, int windowHeight, float fovy) :
    mPos(pos), mPitch(pitch), mYaw(yaw), mFovy(fovy), mStepSize(0.5f),
    mWinWidth(windowWidth), mWinHeight(windowHeight) {}

bool Camera::onKey(int key)
{
    bool ret = false;
    switch(key)
    {
        case 'w':
        {
            mPos += mStepSize * getDir();
            ret = true;
        }
        break;

        case 's':
        {
            mPos -= mStepSize * getDir();
            ret = true;
        }
        break;

        case 'a':
        {
            mPos -= mStepSize * getDir().cross(getUp());
            ret = true;
        }
        break;

        case 'd':
        {
            mPos += mStepSize * getDir().cross(getUp());
            ret = true;
        }
        break;
    }

    return ret;
}

bool Camera::onMouse(int x, int y)
{
    //Increment the yaw of the camera
    int deltaX = x - mWinWidth / 2;
    mYaw += deltaX * 0.02f;
    mYaw = fmod(mYaw, 360.0f);

    //Increment the pitch of the camera
    //Note that window y coordinates increase downwards
    int deltaY = mWinHeight / 2 - y;
    mPitch += deltaY * 0.02f;
    mPitch = clamp(mPitch, -90.0f, 90.0f);

    return true;
}

Eigen::Vector3f Camera::getDir() {
    Eigen::Vector3f dir(0, 0, -1);
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(degToRad(mYaw), -1 * Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(degToRad(mPitch), Eigen::Vector3f::UnitX());

    //Rotate default view direction by yaw and pitch
    return rot * dir;
}

Eigen::Vector3f Camera::getUp() {
    Eigen::Vector3f up(0, 1, 0);
    Eigen::Matrix3f rot;
    rot = Eigen::AngleAxisf(degToRad(mYaw), -1 * Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(degToRad(mPitch), Eigen::Vector3f::UnitX());

    //Rotate default up direction by yaw and pitch
    return rot * up;
}

Eigen::Vector3f Camera::unproject(int x, int y) {
    Eigen::Vector3f dir = getDir();
    Eigen::Vector3f up = getUp();
    Eigen::Vector3f right = dir.cross(up);

    float ar = float(mWinWidth) / mWinHeight;
    float screenHeight = 2 * std::tan(degToRad(mFovy * 0.5f));
    float screenWidth = screenHeight * ar;

    float xNorm = (2.0f * x) / mWinWidth - 1.0f;
    float yNorm = 1.0f - (2.0f * y) / mWinHeight;

    Eigen::Vector3f ray = dir + xNorm * 0.5f * screenWidth * right
                         + yNorm * 0.5f * screenHeight * up;
    ray.normalize();
    return ray;
}
