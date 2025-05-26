#ifndef _INVERSEKINEMATICS_H_
#define _INVERSEKINEMATICS_H_
#include <vector>
#include <cmath>
#include <map>
#include "DataPacket.h"

struct LegConfig {
    double rotationAngle;
    bool isMirrored;
};

struct vector3 {
    double x;
    double y;
    double z;

    vector3 operator+(const vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    vector3 operator-(const vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    vector3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }
    double length() const {
        return sqrt(x * x + y * y + z * z);
    }
};

struct jointAngles {
    double coxaAngle;
    double femurAngle;
    double tibiaAngle;
};

class Calculate {
private:
    static constexpr double coxaLength = 50.50;
    static constexpr double femurLength= 90;
    static constexpr double tibiaLength= 150.35;
    const double pi = acos(-1.0);

public:
    std::vector<double> angle(std::vector<double> position, int legNum);
    std::vector<double> direction(std::vector<double> start);
    std::map<int, LegConfig> legConfigs;

    Calculate() {
        legConfigs[1] = {30 * pi / 180, false};
        legConfigs[2] = {0, false};
        legConfigs[3] = {-30 * pi / 180, false};
        legConfigs[4] = {30 * pi / 180, true};
        legConfigs[5] = {0, true};
        legConfigs[6] = {-30 * pi / 180, true};
    }
};

#endif 