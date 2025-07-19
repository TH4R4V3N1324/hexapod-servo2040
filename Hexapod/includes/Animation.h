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
    bool gaitChangeRequested = false;

public:
    enum Gait {GAIT_TRIPOD, GAIT_RIPPLE, GAIT_WAVE, NUM_GAITS};
    enum Mode {MODE_NORMAL, MODE_STRAFE, MODE_TILT, MODE_CONFIG, NUM_MODES};
    Animation::Gait currentGait;
    Animation::Gait pendingGait;
    Animation::Mode currentMode;
    Animation() : currentGait(GAIT_TRIPOD), currentMode(MODE_NORMAL) {}
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void CycleGait();
    void SetGait(Gait gait);
    void CycleMode();
    void SetMode(Mode mode);
    void Startup();
    void Shutdown();
    void Strafe();
    void returnToStart();
};

#endif 