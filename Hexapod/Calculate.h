#ifndef _INVERSEKINEMATICS_H_
#define _INVERSEKINEMATICS_H_
#include <vector>
#include <cmath>
#include "DataPacket.h"

class Calculate {
private:
    static constexpr double coxaLength = 50.50;
    static constexpr double femurLength= 90;
    static constexpr double tibiaLength= 150.35;

public:
    std::vector<double> angle(std::vector<double> position);
    std::vector<double> direction(std::vector<double> start);
};

#endif 