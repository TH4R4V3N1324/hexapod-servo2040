#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"
#include "Calculate.h"
#include <stdio.h>

struct GaitState {
    std::vector<std::vector<int>> config;
    std::map<int, std::vector<Vector3>> swingTrajectory;
    std::map<int, std::vector<Vector3>> stanceTrajectory;
    int phase = 0;
    int step = 0;
};

class Animation{
private:
    enum Gait {tripod, ripple, wave};
    enum Mode {normal, strafe, tilt};
    Animation::Gait currentGait;
    Animation::Mode currentMode;
    Vector3 homePos {110, 110, 0};
    Vector3 startPos {90, 90, -100};
    Move move;
    Calculate cal;
    GaitState gaitState;

public:
    Animation() : currentGait(tripod), currentMode(normal) {}
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void CycleGait();
    void CycleMode();
    void Startup();
    void Shutdown();
    void Strafe();
};

#endif 