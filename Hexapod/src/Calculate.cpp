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

//returns the end position based on the direction of the joystick and a known distance
Vector3 Calculate::direction(const Vector3& start){
    int distance = 10;

    if (std::abs(receivedData.LStickX) <= 10 && std::abs(receivedData.LStickY) <= 10) {
        return start;
    }

    double joy_x = static_cast<double>(receivedData.LStickX) / 128.0;
    double joy_y = static_cast<double>(receivedData.LStickY) / 128.0;

    double angle = atan2(joy_y, joy_x);
    double deltaX = distance * cos(angle);
    double deltaY = distance * sin(angle);

    double endX = start.x + deltaX;
    double endY = start.y + deltaY;

    return {endX, endY, start.z};
};

//Generates an arc trajectory between the start and end position of a leg
std::vector<Vector3> Calculate::GenerateArcTrajectory(const Vector3& end, bool invert, int legNum, int resolution){
    std::vector<Vector3> trajectory;
    Vector3 start = Calculate::legPosition.at(legNum);
    Vector3 arcCentre {
        (start.x + end.x) / 2, 
        (start.y + end.y) / 2, 
        (start.z + end.z) / 2
    };

    double radius = (sqrt((pow(start.x - end.x, 2)) + (pow(start.y - end.y, 2)))) / 2;

    for(size_t i = 0; i <= resolution; ++i){
        double percentage = static_cast<double>(i) / resolution;
        double x = start.x + (end.x - start.x) * percentage;
        double y = start.y + (end.y - start.y) * percentage;
        double a = sqrt((pow(arcCentre.x - x, 2)) + (pow(arcCentre.y - y, 2)));

        double z;
        if (invert == true){
            z = arcCentre.z - ((radius - a) / 2);
        }
        else{
            z = (radius - a) + arcCentre.z;
        }    
        trajectory.push_back({x, y, z});
    };
    return trajectory;
};