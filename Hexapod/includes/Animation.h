#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"
#include "Calculate.h"
#include <stdio.h>
#include <array>

struct GaitState {
    std::vector<std::vector<int>> config;
    std::array<std::vector<Vector3>, 6> swingTrajectory;
    std::array<std::vector<Vector3>, 6> stanceTrajectory;
    int phase = 0;
    int step = 0;
    bool idleReturning = false;
};

class Animation{
private:
    enum Gait {tripod, ripple, wave, NumGaits};
    enum Mode {normal, strafe, tilt, NumModes};
    Animation::Gait currentGait;
    Animation::Mode currentMode;
    Vector3 homePos {0, 150, 0};
    Vector3 startPos {0, 130, -120};
    Move move;
    Calculate cal;
    GaitState gaitState;
    bool firstStepAfterRest = true;

public:
    Animation() : currentGait(tripod), currentMode(normal) {}
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void CycleGait();
    void CycleMode();
    void Startup();
    void Shutdown();
    void Strafe();
    void returnToStart();
};

#endif 