#include "Animation.h"

//Returns leg configuration based on given gate
std::vector<std::vector<int>> Animation::GetLegConfig(Gait gait){
    switch (gait){
        case GAIT_TRIPOD:
            return {{1, 3, 5}, {2, 4, 6}};
            break;
        case GAIT_RIPPLE:
            return {{3, 6}, {2, 4}, {1, 5}};
            break;
        case GAIT_WAVE:
            return {{3}, {2}, {1}, {4}, {5}, {6}};
            break;
        default:
            return {};
            break;
    }
}

// Changes to the next gait when called
void Animation::CycleGait(){
    pendingGait = static_cast<Gait>((currentGait + 1) % NUM_GAITS);
    gaitChangeRequested = true;
}

// Changes to given gait
void Animation::SetGait(Gait gait) {
    pendingGait = gait;
    gaitChangeRequested = true;
}

// Changes to the next mode when called
void Animation::CycleMode() {
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
}

// Changes to given mode
void Animation::SetMode(Mode mode) {
    currentMode = mode;
}

// Changes currentHeight to new height and updates startPosition
void Animation::SetHeight(double newHeight) {
    hexPacket.currentHeight = newHeight;
    startPos = Vector3{0, 130, -static_cast<double>(hexPacket.currentHeight)};
    startPosition = {
        {1, startPos.rotate(-15)},
        {2, startPos},
        {3, startPos.rotate(15)},
        {4, startPos.rotate(15)},
        {5, startPos},
        {6, startPos.rotate(-15)}
    };
}

// Move to home, deactivate servos
void Animation::Shutdown() {
    for(size_t i = 1; i <= 6; ++i){
        move.Position(homePos, i);
        sleep_ms(1000);
        move.Deactivate(i);
    }
}

// Move to home position for all legs
void Animation::Startup() {
    static bool initialized = false;

    if(!initialized){
        Shutdown();
        initialized = true;
    }

    startPos = Vector3{0, 130, -static_cast<double>(hexPacket.currentHeight)};
    startPosition = {
        {1, startPos.rotate(-15)},
        {2, startPos},
        {3, startPos.rotate(15)},
        {4, startPos.rotate(15)},
        {5, startPos},
        {6, startPos.rotate(-15)}
    };
    
    for(size_t i = 1; i <= 6; ++i){
        move.Position(startPosition.at(i), i);
    }
}

// Returns if stick values have changed significantly
bool SignificantStickChange(const int& stickX, const int& stickY) {
    static int lastX = 0;
    static int lastY = 0;

    // Check if the change in stick position is significant
    bool significantChange = (std::abs(stickX - lastX) > 10 || std::abs(stickY - lastY) > 10);

    // Update the last known positions
    lastX = stickX;
    lastY = stickY;

    return significantChange;
}

Vector3 Animation::BlendTargetPosition(const Vector3& currentPos, const Vector3& forwardPos, const Vector3& rotationPos) {
    // Compute deltas from current position
    Vector3 forwardDelta = forwardPos - currentPos;
    Vector3 rotationDelta = rotationPos - currentPos;

    // Add the deltas
    Vector3 blended = currentPos + forwardDelta + rotationDelta;

    // Optionally, clamp the stride to a maximum distance from currentPos if needed
    double maxStride = 100.0;
    if ((blended - currentPos).length() > maxStride) {
         blended = currentPos + (blended - currentPos).normalized() * maxStride;
    }

    return blended;
}

// Handles idle return logic and returns true if idle return was handled
bool Animation::HandleIdleReturn() {
    static int idleCount = 0;
    static const int idleThreshold = 100;

    // Check if stick is idle
    bool stickIdle = (std::abs(controlPacket.joystick1X) <= 10 && std::abs(controlPacket.joystick1Y) <= 10);
    if (stickIdle) idleCount++;
    else idleCount = 0;

    // Handle idle/return-to-start logic
    if (idleCount > idleThreshold || gaitState.idleReturning) {
        if (!gaitState.idleReturning) {
            gaitState.idleReturning = true;
            gaitState.step = 0;
        }
        returnToStart();
        idleCount = 0;
    }
    return stickIdle; // Return whether stick is idle
}

// Ensures the gait configuration is set up correctly
void Animation::EnsureGaitConfig() {
    if (gaitState.config.empty()) {
        gaitState.config = GetLegConfig(currentGait);
    }
}

// Calculate stride multiplier based on the number of phases in the current gait
double Animation::CalculateStrideMultiplier() {
    return (gaitState.config.size() > 1) ? 1.0 / (gaitState.config.size() - 1) : 1.0;
}

// Generates trajectories for the current gait phase
void Animation::GenerateTrajectories(
    int liftHeight,
    int resolution,
    std::function<Vector3(int, const Vector3&)> swingTargetFunc,
    std::function<Vector3(int, const Vector3&)> stanceTargetFunc
) {
    // Assign legs to their respective swing and stance groups
    auto swingGroup = gaitState.config[gaitState.phase];
    std::vector<int> stanceGroup;
    for (int idx = 0; idx < gaitState.config.size(); ++idx) {
        if (idx == gaitState.phase) continue;
        for (int legNum : gaitState.config[idx])
            stanceGroup.push_back(legNum);
    }
    for (int i = 1; i <= MAX_LEGS; ++i) {
        gaitState.swingSizes[i] = 0;
        gaitState.stanceSizes[i] = 0;
    }

    std::map<int, Vector3> swingTargetsBodyFrame;
    std::map<int, Vector3> stanceTargetsBodyFrame;

    // Calculate swing and stance targets in body frame
    for (int legNum : swingGroup) {
        Vector3 currentPos = move.GetLegPosition(legNum);
        Vector3 targetLegFrame = swingTargetFunc(legNum, currentPos);
        Vector3 targetBodyFrame = cal.convertToBodyFrame(targetLegFrame, legNum);
        swingTargetsBodyFrame[legNum] = targetBodyFrame;
    }
    for (int legNum : stanceGroup) {
        Vector3 currentPos = move.GetLegPosition(legNum);
        Vector3 targetLegFrame = stanceTargetFunc(legNum, currentPos);
        Vector3 targetBodyFrame = cal.convertToBodyFrame(targetLegFrame, legNum);
        stanceTargetsBodyFrame[legNum] = targetBodyFrame;
    }

    // Collision check and adjustment
    double threshold = 50.0; // mm
    for (int legNum : swingGroup) {
        Vector3 swingTargetBody = swingTargetsBodyFrame[legNum];
        for (const auto& [stanceNum, stanceTargetBody] : stanceTargetsBodyFrame) {
            if ((swingTargetBody - stanceTargetBody).length() < threshold) {
                // Clamp swingTargetBody outward
                Vector3 dir = (swingTargetBody - stanceTargetBody).normalized();
                swingTargetBody = stanceTargetBody + dir * threshold;
            }
        }
        // Convert back to leg frame
        Vector3 targetLegFrame = cal.convertToLegFrame(swingTargetBody, legNum);
        int size = 0;
        cal.GenerateBezierTrajectory(
            gaitState.swingTrajectory[legNum].data(),
            size,
            move.GetLegPosition(legNum),
            targetLegFrame,
            liftHeight,
            resolution
        );
        gaitState.swingSizes[legNum] = size;
    }

    // Stance
    for (int legNum : stanceGroup) {
        Vector3 targetLegFrame = cal.convertToLegFrame(stanceTargetsBodyFrame[legNum], legNum);
        int size = 0;
        cal.GenerateStraightTrajectory(
            gaitState.stanceTrajectory[legNum].data(),
            size,
            move.GetLegPosition(legNum),
            targetLegFrame,
            resolution
        );
        gaitState.stanceSizes[legNum] = size;
    }
}

// Performs a single step for all legs based on the current gait state
void Animation::PerformLegStep(bool stickIdle, int resolution, bool handlePhaseTransition) {
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int swingSize = gaitState.swingSizes[legNum];
        int stanceSize = gaitState.stanceSizes[legNum];
        if (gaitState.step < swingSize)
            move.Position(gaitState.swingTrajectory[legNum][gaitState.step], legNum);
        if (gaitState.step < stanceSize)
            move.Position(gaitState.stanceTrajectory[legNum][gaitState.step], legNum);
    }
    // Advance step if not idle
    if (!stickIdle) gaitState.step++;
    if (!handlePhaseTransition) return;

    // Phase transition
    if (gaitState.step > resolution) {
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size();
        gaitState.step = 0;
    }    
}

// Returns to the start position of the hexapod
void Animation::returnToStart() {
    static int counter = 0;
    static bool trajectoryGenerated = false;
    int liftHeight = 50;
    int resolution = 50;

    // Safety check: phase must be valid
    if (gaitState.phase >= gaitState.config.size()) {
        std::cerr << "[returnToStart] ERROR: Invalid gaitState.phase: "
                  << gaitState.phase << ", config size: "
                  << gaitState.config.size() << std::endl;
        gaitState.idleReturning = false;
        counter = 0;
        gaitState.step = 0;
        trajectoryGenerated = false;
        return;
    }

    if (gaitState.step == 0 && !trajectoryGenerated) {
        GenerateTrajectories(
            liftHeight,
            resolution,
            // Swing: move to start position
            [this](int legNum, const Vector3& currentPos) {
                auto it = startPosition.find(legNum);
                return (it != startPosition.end()) ? it->second : currentPos;
            },
            // Stance: hold current position
            [](int, const Vector3& currentPos) {
                return currentPos;
            }
        );
    }

    // Move all legs for this step
    PerformLegStep(false, resolution, false);

    // Phase transition
    if (gaitState.step > resolution) {
        counter++;
        gaitState.step = 0;
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size();

        // After all phases, finish return-to-start and handle gait change if requested
        if (counter > gaitState.config.size()) {
            counter = 0;
            gaitState.idleReturning = false;
            trajectoryGenerated = false;

            if (gaitChangeRequested) {
                currentGait = pendingGait;
                gaitState.config = GetLegConfig(currentGait);
                gaitState.phase = 0;
                gaitState.step = 0;
                gaitChangeRequested = false;
            }
        }
    }
}

// Handles the configuration state for the hexapod
void Animation::ConfigState() {
    JointAngles angles = {0, 90, 0}; // Default angles for configuration state
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        move.Angles(angles, legNum);
    }
}

void Animation::Strafe() {
    int liftHeight = 50;
    int resolution = 50;

    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (gaitState.idleReturning) return;

    // Ensure gait config is set
    EnsureGaitConfig();

    // Calculate stride multiplier safely
    double strideMultiplier = CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (gaitState.step == 0) {
        GenerateTrajectories(
        liftHeight,
        resolution,
        // Swing target
        [this](int legNum, const Vector3& currentPos) {
            Vector3 directionPos = cal.direction(controlPacket.joystick1X, controlPacket.joystick1Y, currentPos, legNum);
            Vector3 rotationPos = cal.direction(controlPacket.joystick2X, 0, currentPos, legNum, false, 1.0, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, directionPos, rotationPos);
            return targetPos;
        },
        // Stance target
        [this, strideMultiplier](int legNum, const Vector3& currentPos) {
            Vector3 directionPos = cal.direction(controlPacket.joystick1X, controlPacket.joystick1Y, currentPos, legNum, true, strideMultiplier);
            Vector3 rotationPos = cal.direction(controlPacket.joystick2X, 0, currentPos, legNum, false, strideMultiplier, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, directionPos, rotationPos);
            return targetPos;
        }
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, resolution);
}

void Animation::Normal() {
    int liftHeight = 50;
    int resolution = 50;

    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (gaitState.idleReturning) return;

    // Ensure gait config is set
    EnsureGaitConfig();

    // Calculate stride multiplier safely
    double strideMultiplier = CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (gaitState.step == 0) {
        GenerateTrajectories(
        liftHeight,
        resolution,
        // Swing target
        [this](int legNum, const Vector3& currentPos) {
            Vector3 forwardPos = cal.direction(0, controlPacket.joystick1Y, currentPos, legNum);
            Vector3 rotationPos = cal.direction(controlPacket.joystick1X, 0, currentPos, legNum, false, 1.0, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, forwardPos, rotationPos);
            return targetPos;
        },
        // Stance target
        [this, strideMultiplier](int legNum, const Vector3& currentPos) {
            Vector3 forwardPos = cal.direction(0, controlPacket.joystick1Y, currentPos, legNum, true , strideMultiplier);
            Vector3 rotationPos = cal.direction(controlPacket.joystick1X, 0, currentPos, legNum, true, strideMultiplier, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, forwardPos, rotationPos);
            return targetPos;
        }
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, resolution);
} 