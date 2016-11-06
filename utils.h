#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <sstream>
#include <string.h>

//String functions
bool ReadFile(const char* pFileName, std::string& outFile);
std::string numToStr(int n);

//Math functions
float degToRad(float deg);
float clamp(float value, float lower, float higher);
#endif
