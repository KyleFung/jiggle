#include "utils.h"

#include <unsupported/Eigen/Polynomials>

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

bool approx(float a, float b, float e) {
    return (a - b) * (a - b) < e * e;
}

Eigen::Vector3f bary(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c,
                     Eigen::Vector3f p)
{
    Eigen::Vector3f v0 = b - a;
    Eigen::Vector3f v1 = c - a;
    Eigen::Vector3f v2 = p - a;

    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);

    float norm = d00 * d11 - d01 * d01;
    if(norm == 0) {
        throw norm;
    }

    float v = (d11 * d20 - d01 * d21) / norm;
    float w = (d00 * d21 - d01 * d20) / norm;
    float u = 1.0f - v - w;

    return Eigen::Vector3f(v, w, u);
}

Eigen::Vector2f intersect(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f d) {
    Eigen::Vector3f x = (b - a).cross(d - c);
    Eigen::Vector3f y = (c - a).cross(d - c);

    float t = y.dot(x) / x.dot(x);

    Eigen::Vector3f p = a + t * (b - a);
    float s = (p - c).dot(d - c) / (d - c).dot(d - c);

    return Eigen::Vector2f(t, s);
}

float smallestPosRealRoot(float a0, float a1, float a2, float a3)
{
    float smallest = 10000;

    int degree = 3;
    if(approx(a3, 0, 0.0001)) {
        degree = 2;
        if(approx(a2, 0, 0.0001)) {
            degree = 1;
        }
    }

    if(degree == 3) {
        // Use eigen solver because I can't implement cardano's...
        Eigen::Matrix<float,4,1> poly;
        poly << a0, a1, a2, a3;
        Eigen::PolynomialSolver<float,3> psolvef(poly);
        Eigen::Vector3cf roots = psolvef.roots();
        for(int i = 0; i < degree; i++) {
            // Check if the root is real
            if(approx(roots[i].imag(), 0, 0.0001)) {
                // Check if it's in [0,h] and smaller than current min
                float real = roots[i].real();
                if(real >= 0 && real < smallest && !isnan(real)) {
                    smallest = real;
                }
            }
        }
    }
    else if(degree == 2) {
        // Use quadratic formula here
        float det = a1 * a1 - 4 * a2 * a0;
        if(det < 0) {
            return -1;
        }
        Eigen::Vector2f roots;
        roots[0] = ((-1 * a1) + std::sqrt(det)) / (2 * a2);
        roots[1] = ((-1 * a1) - std::sqrt(det)) / (2 * a2);
        for(int i = 0; i < degree; i++) {
            // Check if it's in [0,h] and smaller than current min
            float real = roots[i];
            if(real >= 0 && real < smallest && !isnan(real)) {
                smallest = real;
            }
        }
    }
    else if(degree == 1) {
        // Formula for a line
        float real = -1.0f * (a0 / a1);
        if(real >= 0 && real < smallest && !isnan(real)) {
            smallest = real;
        }
    }

    // If none fit that critera, then there is no collision
    if(smallest == 10000) {
        return -1;
    }

    return smallest;
}
