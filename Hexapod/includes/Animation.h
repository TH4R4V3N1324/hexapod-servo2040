#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"

class Animation{
private:
enum Gait {tripod, ripple, wave};

public:
std::vector<std::vector<int>> GetLegConfig(Gait gait);
void Startup();
void Shutdown();
void Walk(std::vector<double> start, std::vector<double> end);
void Rotate();
};

#endif 