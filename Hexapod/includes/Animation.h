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
    std::vector<double> startPos {120, 120, -100};

public:
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void SetGait(Gait gait);
    void SetMode(Mode mode);
    void CycleGait(Animation &animation);
    void CycleMode(Animation &animation);
    void Startup();
    void Shutdown();
    void Strafe();
};

#endif 