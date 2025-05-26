#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"
#include "Calculate.h"
#include <stdio.h>

class Animation{
private:
    enum Gait {tripod, ripple, wave};
    enum Mode {normal, strafe, tilt};
    Animation::Gait currentGait;
    Animation::Mode currentMode;
    Vector3 homePos {110, 110, 0};
    Vector3 startPos {120, 120, -100};
    Move move;
    Calculate cal;
    Animation() : currentGait(tripod), currentMode(normal) {}

public:
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void CycleGait();
    void CycleMode();
    void Startup();
    void Shutdown();
    void Strafe();
};

#endif 