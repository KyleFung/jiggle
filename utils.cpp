#include "utils.h"

//Takes a file name and spits the contents into a string
bool ReadFile(const char* pFileName, std::string& outFile)
{
    std::ifstream f(pFileName);

    bool ret = false;
    
    if (f.is_open()) {
        std::string line;
        while (getline(f, line)) {
            outFile.append(line);
            outFile.append("\n");
        }

        f.close();

        ret = true;
    }
    else {
        printf("Failed to open file\n");
    }
    return ret;
}

std::string numToStr(int n)
{
    std::stringstream ss;
    ss << n;
    return ss.str();
}

float degToRad(float deg)
{
    return deg * 0.0174532925f;
}

float clamp(float value, float lower, float higher)
{
    if(value < lower)
        return lower;
    if(value > higher)
        return higher;
    return value;
}
