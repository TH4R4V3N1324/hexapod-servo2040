#include "Move.h"

using namespace std;

unordered_map<int, Vector3> Move::legPosition;

Move::Move(){
    SetupSwitches();
}

//sets up the switches with pull down resistors if not setup already
void Move::SetupSwitches(){
    static bool initialized = false;

    if (!initialized) {
        for (const auto& pair : legSwitch) {
            mux.configure_pulls(pair.second, false, true);
        }
        initialized = true;
    }   
}

//returns state of switch found in legSwitch
bool Move::GetSwitchStatus(int legNum){
    mux.select(legSwitch.at(legNum));
    return mux.read();
}

//
bool Move::AllLegsGrounded(){
    for (size_t i = 1; i < legSwitch.size(); ++i) {
        if (!Move::GetSwitchStatus(i)) {
            return false;
        }
    }

    return true;
}

//returns leg position found in legPosition if it exists
Vector3 Move::GetLegPosition(int legNum) const{
    if (legPosition.find(legNum) != legPosition.end()) {
        return legPosition.at(legNum);
    } else {
        return {};
    }
}

//moves leg tip to position through coordinates
void Move::Coordinate(double x, double y, double z, int legNum){
    LegServo Servos = legs.at(legNum);
    JointAngles angles = Cal.angle({x, y, z}, legNum);
    
    // Assign angles to servos directly (assuming 3 servos per leg)
    cluster.value(Servos.coxa, angles.coxaAngle);
    cluster.value(Servos.femur, angles.femurAngle);
    cluster.value(Servos.tibia, angles.tibiaAngle);
    
    legPosition[legNum] = {x, y, z};
};

//moves leg tip to position through a vector
void Move::Position(const Vector3& position, int legNum){
    LegServo Servos = legs.at(legNum);
    JointAngles angles = Cal.angle(position, legNum);

    // Assign angles to servos directly (assuming 3 servos per leg)
    cluster.value(Servos.coxa, angles.coxaAngle);
    cluster.value(Servos.femur, angles.femurAngle);
    cluster.value(Servos.tibia, angles.tibiaAngle);

    legPosition[legNum] = position;
};

//moves leg tip in straight line from start to end
void Move::StraightLine(const Vector3& end, int legNum){
    Vector3 start = legPosition.at(legNum);

    int resolution = 12;
    for(size_t i = 0; i <= resolution; i++){
        double percentage = static_cast<double>(i) / resolution;
        double x = start.x + (end.x - start.x) * percentage;
        double y = start.y + (end.y - start.y) * percentage;
        double z = start.z + (end.z - start.z) * percentage;
        Position({x, y, z}, legNum);
    }
};

//moves leg tip in an arc from start to end, can invert arc direction
void Move::Arc(const Vector3& end, bool invert, int legNum){
    Vector3 start = legPosition.at(legNum);
    Vector3 arcCentre {
        (start.x + end.x) / 2, 
        (start.y + end.y) / 2, 
        (start.z + end.z) / 2
    };

    int resolution = 10;
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
        Position ({x, y, z}, legNum);
    };
};

//turns the servos off in a given leg
void Move::Deactivate(int legNum){
    LegServo Servos = legs.at(legNum);

    cluster.disable(Servos.coxa);
    cluster.disable(Servos.femur);
    cluster.disable(Servos.tibia);
};