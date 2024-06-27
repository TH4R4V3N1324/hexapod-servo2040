#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"

class Animation{
private:
    enum Gait {tripod, ripple, wave};
    enum Mode {normal, strafe, tilt};
    Animation::Gait currentGait;
    Animation::Mode currentMode;

public:
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void SetGait(Gait gait);
    void SetMode(Mode mode);
    void CycleGait(Animation &animation);
    void CycleMode(Animation &animation);
    void Startup();
    void Shutdown();
    void Walk(std::vector<double> start, std::vector<double> end);
    void Rotate();
};

#endif 