#include "Calculate.h"
using namespace std;

unordered_map<int, Vector3> Calculate::legPosition;

//inverse kinematics, returns the angles needed to move to position
JointAngles Calculate::angle(Vector3 position, int legNum){
    LegConfig config = legConfigs[legNum];

    if(config.rotationAngle != 0){
        double xNew = position.x * cos(-config.rotationAngle) - position.y * sin(-config.rotationAngle);
        double yNew = position.x * sin(-config.rotationAngle) + position.y * cos(-config.rotationAngle);
        position.x = xNew;
        position.y = yNew;
    }

    if(config.isMirrored){
        position.y = -position.y;
        position.x = -position.x;
    }
     
    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle = (atan2(position.x, position.y) * (180/pi))-45;
    double r1 = std::hypot(position.x, position.y) - a1;
    double r2 = position.z;
    double q2 = atan(r2/r1) * (180/pi);
    double r3 = std::hypot(r1, r2);
    double q1 = acos((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)) * (180/pi);
    double femurAngle = (q2+q1);
    double q3 = acos((pow(r3,2) - pow(a2,2) - pow(a3,2)) / (-2*a2*a3)) * (180/pi);
    double tibiaAngle = 90 - q3;

    return {coxaAngle, femurAngle, tibiaAngle};
};

//Calculates end position of a leg based on the current position, velocity, and time delta
Vector3 Calculate::direction(const Vector3& start, double velocity, double dt) {
    const double maxStride = 20.0; // Maximum stride length

    double stickX = static_cast<double>(receivedData.LStickX);
    double stickY = static_cast<double>(receivedData.LStickY);

    //Check if the stick is within a dead zone
    if (std::abs(stickX) <= 10 && std::abs(stickY) <= 10) {
        return start;
    }

    //Calculate stick magnitude and clamp to 1.0
    double magnitude = std::hypot(stickX, stickY) / 128.0;
    if (magnitude > 1.0) magnitude = 1.0;

    //Dynamic stride based on velocity, dt, and stick magnitude
    double stride = std::min(velocity * dt * magnitude, maxStride);

    //Calculate direction angle
    double angle = atan2(stickY, stickX);

    //Compute deltas
    double deltaX = stride * cos(angle);
    double deltaY = stride * sin(angle);

    return {start.x + deltaX, start.y + deltaY, start.z};
}

//Generates an arc trajectory between the start and end position of a leg
std::vector<Vector3> Calculate::GenerateArcTrajectory(const Vector3& start, const Vector3& end, int liftHeight, int resolution){
    std::vector<Vector3> trajectory;

    //Linear interpolation for x and y
    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        double x = start.x + (end.x - start.x) * t;
        double y = start.y + (end.y - start.y) * t;

        //Sine-based arc for z
        double z = start.z + (end.z - start.z) * t + liftHeight * std::sin(M_PI * t);
        trajectory.push_back({x, y, z});
    }
    return trajectory;
};