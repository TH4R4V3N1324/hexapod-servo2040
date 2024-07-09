#include "Move.h"

using namespace std;

unordered_map<int, vector<double>> Move::legPosition;

Move::Move(){
    SetupSwitches();
}

//sets up the switches with pull down resistors if not setup already
void Move::SetupSwitches(){
    static bool initialized = false;

    if (!initialized) {
        for(size_t i = 0; i < legSwitch.size(); i++) {
            mux.configure_pulls(legSwitch[1] + i, false, true);
        }
        initialized = true;
    }
}

//returns state of switch found in legSwitch
bool Move::GetSwitchStatus(int legNum){
    mux.select(legSwitch[legNum]);
    return mux.read();
}

//
bool Move::AllLegsGrounded(){
    for (size_t i = 0; i < legSwitch.size(); ++i) {
        if (!Move::GetSwitchStatus(i)) {
            return false;
        }
    }

    return true;
}

//returns leg position found in legPosition if it exists
std::vector<double> Move::GetLegPosition(int legNum){
    if (legPosition.find(legNum) != legPosition.end()) {
        return legPosition[legNum];
    } else {
        return {};
    }
}

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

    legPosition[legNum] = position;
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

    legPosition[legNum] = position;
};

//moves leg tip in straight line from start to end
void Move::StraightLine(vector<double> end, int legNum){
    std::vector<double> start = legPosition[legNum];

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
void Move::Arc(vector<double> end, bool invert, int legNum){
    std::vector<double> start = legPosition[legNum];

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
        else{
            double z = (radius - a) + arcCentre[2];
        }    
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