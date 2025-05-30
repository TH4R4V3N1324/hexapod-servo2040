#include "Calculate.h"
using namespace std;

unordered_map<int, Vector3> Calculate::legPosition;

//inverse kinematics, returns the angles needed to move to position
JointAngles Calculate::angle(Vector3 position, int legNum){
    LegConfig config = legConfigs[legNum];

    if(config.isMirrored){
        position.x = -position.x; // Mirror the x-coordinate for mirrored legs
    }

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle = (atan2(position.x, position.y) * (180/pi));
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

//Calculates end position of a leg based on the current position, velocity, and time between steps
Vector3 Calculate::direction(const Vector3& start, int legNum, bool invert) {
    const double maxStride = 60.0;

    double stickX = static_cast<double>(receivedData.LStickX);
    double stickY = static_cast<double>(receivedData.LStickY);

    std::swap(stickX, stickY); // Swap X and Y to match the leg's coordinate system

    if (invert) {
        stickX = -stickX;
        stickY = -stickY;
    }

    if (legConfigs[legNum].isMirrored) {
        stickY = -stickY;
    }

    if (std::abs(stickX) <= 10 && std::abs(stickY) <= 10) {return start;}
   
    double magnitude = std::hypot(stickX, stickY) / 128.0;
    if (magnitude > 1.0) magnitude = 1.0;

    double stride = maxStride * magnitude;

    double angle = atan2(stickY, stickX);

    double rotationAngle = legConfigs[legNum].rotationAngle;
    double deltaX = stride * cos(angle);
    double deltaY = stride * sin(angle);

    double dx_rot = deltaX * cos(rotationAngle) - deltaY * sin(rotationAngle);
    double dy_rot = deltaX * sin(rotationAngle) + deltaY * cos(rotationAngle);

    return {start.x + dx_rot, start.y + dy_rot, start.z};
}

//Generates an arc trajectory between the start and end position
std::vector<Vector3> Calculate::GenerateArcTrajectory(const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert) {
    std::vector<Vector3> trajectory;

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x - end.x) < 1e-6 &&
        std::abs(start.y - end.y) < 1e-6 &&
        std::abs(start.z - end.z) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory.push_back(start);
        }
        return trajectory;
    }

    //Linear interpolation for x and y
    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        double x = start.x + (end.x - start.x) * t;
        double y = start.y + (end.y - start.y) * t;

        //Sine-based arc for z
        double z;
        if (invert) {
            z = start.z + (end.z - start.z) * t - liftHeight * std::sin(M_PI * t);
        } else {
            z = start.z + (end.z - start.z) * t + liftHeight * std::sin(M_PI * t);
        }
        trajectory.push_back({x, y, z});
    }
    return trajectory;
};

//Generates a straight trajectory between the start and end position
std::vector<Vector3> Calculate::GenerateStraightTrajectory(const Vector3& start, const Vector3& end, int resolution){
    std::vector<Vector3> trajectory;

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x - end.x) < 1e-6 &&
        std::abs(start.y - end.y) < 1e-6 &&
        std::abs(start.z - end.z) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory.push_back(start);
        }
        return trajectory;
    }

    for(size_t i = 0; i <= resolution; i++){
        double percentage = static_cast<double>(i) / resolution;
        double x = start.x + (end.x - start.x) * percentage;
        double y = start.y + (end.y - start.y) * percentage;
        double z = start.z + (end.z - start.z) * percentage;
        trajectory.push_back({x, y, z});
    }
    return trajectory;
};