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
Vector3 Calculate::direction(const Vector3& start, int legNum, bool invert, double strideMultiplier) {
    const double maxStride = 60.0;

    double stickX = static_cast<double>(controlPacket.joystick1X);
    double stickY = static_cast<double>(controlPacket.joystick1Y);

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

    double stride = maxStride * magnitude * strideMultiplier;

    double angle = atan2(stickY, stickX);

    double rotationAngle = legConfigs[legNum].rotationAngle;
    double deltaX = stride * cos(angle);
    double deltaY = stride * sin(angle);

    double dx_rot = deltaX * cos(rotationAngle) - deltaY * sin(rotationAngle);
    double dy_rot = deltaX * sin(rotationAngle) + deltaY * cos(rotationAngle);

    return {start.x + dx_rot, start.y + dy_rot, start.z};
}

//Generates an arc trajectory between the start and end position
void Calculate::GenerateArcTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x - end.x) < 1e-6 &&
        std::abs(start.y - end.y) < 1e-6 &&
        std::abs(start.z - end.z) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    //Linear interpolation for x and y
    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        trajectory[i] = {
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t + (invert ? -1 : 1) * liftHeight * std::sin(M_PI * t)
        };
    }
    outSize = resolution + 1;
}

//Generates a straight trajectory between the start and end position
void Calculate::GenerateStraightTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int resolution) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x - end.x) < 1e-6 &&
        std::abs(start.y - end.y) < 1e-6 &&
        std::abs(start.z - end.z) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    for (int i = 0; i <= resolution; i++) {
        double t = static_cast<double>(i) / resolution;
        trajectory[i] = {
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t
        };
    }
    outSize = resolution + 1;
}

void Calculate::GenerateBezierTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    Vector3 dir = end - start;
    if (dir.length() == 0.0) {
        // Stationary case: generate flat path
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    Vector3 dirNorm = dir.normalized();
    double offsetScale = dir.length() * 0.25; // tweak as needed

    // Place control points before start and after end along the movement direction
    Vector3 P0 = start;
    Vector3 P3 = end;
    Vector3 P1 = start - dirNorm * offsetScale;
    Vector3 P2 = end + dirNorm * offsetScale;

    P1.z = start.z + liftHeight;
    P2.z = end.z + liftHeight;

    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        double u = 1.0 - t;

        Vector3 point =
            P0 * (u * u * u) +
            P1 * (3 * u * u * t) +
            P2 * (3 * u * t * t) +
            P3 * (t * t * t);

        trajectory[i] = point;
    }
    outSize = resolution + 1;
}