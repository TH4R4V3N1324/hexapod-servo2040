#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"

class Animation{
private:
    enum Gait {tripod, ripple, wave};
    Animation::Gait currentGait;

public:
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void SetGait(Gait gait);
    void CycleGait(Animation &animation);
    void Startup();
    void Shutdown();
    void Walk(std::vector<double> start, std::vector<double> end);
    void Rotate();
};

#endif 