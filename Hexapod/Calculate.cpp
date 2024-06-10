#include "Calculate.h"
using namespace std;

//inversekinematics, returns the angles needed to move to position
vector<double> Calculate::angle(vector<double> position){
    double x{position[0]}, y{position[1]}, z{position[2]};
     
    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double pi = acos(-1.0);
    double coxaAngle = (atan2(x, y) * (180/pi))-45;
    double r1 = sqrt((pow(x,2) + pow(y,2))) - a1;
    double r2 = z;
    double q2 = atan(r2/r1) * (180/pi);
    double r3 = sqrt(pow(r1,2) + pow(r2,2));
    double q1 = acos((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)) * (180/pi);
    double femurAngle = (q2+q1);
    double q3 = acos((pow(r3,2) - pow(a2,2) - pow(a3,2)) / (-2*a2*a3)) * (180/pi);
    double tibiaAngle = 90 - q3;

    vector<double> angles{coxaAngle, femurAngle, tibiaAngle};
    return angles;
};

//returns the end position based on the direction of the joystick and a known distance
vector<double> Calculate::direction(vector<double> start){
    int distance = 10;
    double startX = start[0];
    double startY = start[1];
    double joy_x = recievedData.LStickX / 128;
    double joy_y = recievedData.LStickY / 128;

    double angle = atan2(joy_y, joy_x);
    double deltaX = distance * cos(angle);
    double deltaY = distance * sin(angle);

    double endX = startX + deltaX;
    double endY = startY + deltaY;

    vector<double> end{endX, endY, start[2]};
    return end;
};
