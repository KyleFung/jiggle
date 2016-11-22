#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string.h>

//String functions
bool ReadFile(const char* pFileName, std::string& outFile);
std::string numToStr(int n);

//Math functions
float degToRad(float deg);
float clamp(float value, float lower, float higher);
bool approx(float a, float b, float e);
Eigen::Vector3f bary(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c,
                     Eigen::Vector3f p);
float smallestPosRealRoot(float a0, float a1, float a2, float a3);
#endif
