#include "Move.h"

using namespace std;

//assigns the relavant servos to their corrosponding leg
map<int, vector<int>> legs = {
        {1, {0, 1, 2}},
        {2, {3, 4, 5}},
        {3, {6, 7, 8}},
        {4, {9, 10, 11}},
        {5, {12, 13, 14}},
        {6, {15, 16, 17}}
    };

//moves leg tip to position through coordinates
void Move::Coordinate(double x, double y, double z, int legNum){
    vector<int> legServos = legs[legNum];
    
    Calculate Cal;
    vector<double>position{x,y,z};
    vector<double>angles {Cal.angle(position)};
    
    for(size_t i = 0; i < legServos.size() && i < angles.size(); ++i){
        int servo = legServos[i];
        int angle = angles[i];
        cluster.value(servo, angle);
    }
};

//moves leg tip to position through a vector
void Move::Position(vector<double> position, int legNum){
    vector<int> legServos = legs[legNum];
    
    Calculate Cal;
    vector<double>angles {Cal.angle(position)};

    for(size_t i = 0; i < legServos.size() && i < angles.size(); ++i){
        int servo = legServos[i];
        int angle = angles[i];
        cluster.value(servo, angle);
    }
};

//moves leg tip in straight line from start to end
void Move::StraightLine(vector<double> start, vector<double> end, int legNum){
    Position(start, legNum);
    sleep_ms(200);

    int resolution = 12;
    for(size_t i = 0; i < resolution + 1; i++){
        double percentage = i / resolution;
        double x = start[0] + (end[0] - start[0]) * percentage;
        double y = start[1] + (end[1] - start[1]) * percentage;
        double z = start[2] + (end[2] - start[2]) * percentage;
        vector<double> target {x, y, z};
        Position(target, legNum);
    }
};

//moves leg tip in an arc from start to end, can invert arc direction
void Move::Arc(vector<double> start, vector<double> end, bool invert, int legNum){
    double x = (start[0] + end[0]) / 2;
    double y = (start[1] + end[1]) / 2;
    double z = (start[2] + end[2]) / 2;
    vector<double> arcCentre {x, y, z};

    int resolution = 10;
    double radius = (sqrt((pow(start[0] - end[0], 2)) + (pow(start[1] - end[1], 2)))) / 2;

    for(size_t i = 0; i < resolution + 1; ++i){
        double percentage = i / resolution;
        double x = start[0] + (end[0] - start[0]) * percentage;
        double y = start[1] + (end[1] - start[1]) * percentage;
        double a = sqrt((pow(arcCentre[0] - x, 2)) + (pow(arcCentre[1] - y, 2)));

        if (invert == true){
            double z = arcCentre[2] - ((radius - a) / 2);
        }
        else
            double z = (radius - a) + arcCentre[2];
        };

        vector<double> target {x, y, z};
        Position (target, legNum);
        sleep_ms(50);
};

//turns the servos off in a given leg
void Move::Deactivate(int legNum){
    vector<int> servos {legs[legNum]};

    for(size_t i = 0; i < servos.size() + 1; ++i){
        cluster.disable(servos[i]);
    };
};