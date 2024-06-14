#ifndef _MOVE_H_
#define _MOVE_H_
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "Calculate.h"
#include "DataPacket.h"
#include <map>
#include <vector>
#include <cmath>

class Move{
private:
    const uint START_PIN = servo::servo2040::SERVO_1;
    const uint END_PIN = servo::servo2040::SERVO_18;
    const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
    servo::ServoCluster cluster = servo::ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    enum Gait {tripod, ripple, wave};

    //assigns the relavant servos to their corrosponding leg
    std::map<int, std::vector<int>> legs = {
        {1, {0, 1, 2}},
        {2, {3, 4, 5}},
        {3, {6, 7, 8}},
        {4, {9, 10, 11}},
        {5, {12, 13, 14}},
        {6, {15, 16, 17}}
    };

public:
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void Coordinate(double x, double y, double z, int legNum);
    void Position(std::vector<double> position, int legNum);
    void StraightLine(std::vector<double> start, std::vector<double> end, int legNum);
    void Arc(std::vector<double> start, std::vector<double> end, bool invert, int legNum);
    void Deactivate(int legNum);
};

#endif 