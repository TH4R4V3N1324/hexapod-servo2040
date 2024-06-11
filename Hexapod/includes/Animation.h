#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"

class Animation{
public:
void Startup();
void Shutdown();
void Walk(std::vector<double> start, std::vector<double> end);
void Rotate();
};

#endif 