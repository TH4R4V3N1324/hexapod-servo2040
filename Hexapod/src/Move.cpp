#include "Move.h"

using namespace std;

Move::Move(){
    SetupSwitches();
    cluster.init();
}

float Move::GetCurrentDraw() {
    mux.select(servo::servo2040::CURRENT_SENSE_ADDR);
    float current = cur_adc.read_current();
    return current;
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
    for (size_t i = 1; i <= legSwitch.size(); ++i) {
        if (!Move::GetSwitchStatus(i)) {
            return false;
        }
    }

    return true;
}

//returns leg position found in legPosition if it exists
Vector3 Move::GetLegPosition(int legNum) const{
    if (Calculate::legPosition.find(legNum) != Calculate::legPosition.end()) {
        return Calculate::legPosition.at(legNum);
    } else {
        return {};
    }
}

//moves leg tip to position through a vector
void Move::Position(const Vector3& position, int legNum){
    LegServo Servos = legs.at(legNum);
    JointAngles angles = Cal.angle(position, legNum);

    // Assign angles to servos directly (assuming 3 servos per leg)
    cluster.value(Servos.coxa, angles.coxaAngle);
    cluster.value(Servos.femur, angles.femurAngle);
    cluster.value(Servos.tibia, angles.tibiaAngle);

    Calculate::legPosition[legNum] = position;
};

//turns the servos off in a given leg
void Move::Deactivate(int legNum){
    LegServo Servos = legs.at(legNum);

    cluster.disable(Servos.coxa);
    cluster.disable(Servos.femur);
    cluster.disable(Servos.tibia);
};

// Sets leg joint to specific angle
void Move::Joint(int legNum, int joint, double angle) {
    LegServo Servos = legs.at(legNum);
    int legJoints[3] = {Servos.coxa, Servos.femur, Servos.tibia};
    if (joint >= 0 && joint < 3) {
        cluster.value(legJoints[joint], angle);
    }
}