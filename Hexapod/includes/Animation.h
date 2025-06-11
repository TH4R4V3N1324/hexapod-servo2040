#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"
#include "Calculate.h"
#include <stdio.h>
#include <array>

static constexpr int MAX_LEGS = 6;
static constexpr int MAX_RESOLUTION = 50 + 1; // +1 for inclusive endpoint

struct GaitState {
    std::vector<std::vector<int>> config;
    std::array<std::array<Vector3, MAX_RESOLUTION>, MAX_LEGS + 1> swingTrajectory;  // 1-based indexing
    std::array<std::array<Vector3, MAX_RESOLUTION>, MAX_LEGS + 1> stanceTrajectory;
    std::array<int, MAX_LEGS + 1> swingSizes{};   // Store actual size for each leg
    std::array<int, MAX_LEGS + 1> stanceSizes{};
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
    std::map<int, Vector3> startPosition{
        {1, startPos.rotate(-15)},
        {2, startPos},
        {3, startPos.rotate(15)},
        {4, startPos.rotate(15)},
        {5, startPos},
        {6, startPos.rotate(-15)}
    };
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