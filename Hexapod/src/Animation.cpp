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

    auto swingGroup = gaitState.config[gaitState.phase];
    std::vector<int> stanceGroup;

    // Build stance group (all legs not in swing group)
    for (int idx = 0; idx < gaitState.config.size(); ++idx) {
        if (idx == gaitState.phase) continue;
        for (int legNum : gaitState.config[idx])
            stanceGroup.push_back(legNum);
    }

    // Generate trajectories at the start of each phase
    if (gaitState.step == 0 && !trajectoryGenerated) {
        // Clear sizes for all legs
        for (int i = 1; i <= MAX_LEGS; ++i) {
            gaitState.swingSizes[i] = 0;
            gaitState.stanceSizes[i] = 0;
        }

        // Generate swing trajectories (to start positions)
        for (int legNum : swingGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            int size = 0;
            auto it = startPosition.find(legNum);
            if (it == startPosition.end()) {
                printf("startPosition missing for legNum: %d\n", legNum);
                continue;
            }
            cal.GenerateBezierTrajectory(
                gaitState.swingTrajectory[legNum].data(),
                size,
                currentPos,
                it->second,
                liftHeight,
                resolution
            );
            gaitState.swingSizes[legNum] = size;
        }

        // Generate stance trajectories (hold current position)
        for (int legNum : stanceGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            for (int i = 0; i <= resolution; ++i)
                gaitState.stanceTrajectory[legNum][i] = currentPos;
            gaitState.stanceSizes[legNum] = resolution + 1;
        }
    }

    // Move all legs for this step
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int swingSize = gaitState.swingSizes[legNum];
        int stanceSize = gaitState.stanceSizes[legNum];
        if (gaitState.step < swingSize)
            move.Position(gaitState.swingTrajectory[legNum][gaitState.step], legNum);
        if (gaitState.step < stanceSize)
            move.Position(gaitState.stanceTrajectory[legNum][gaitState.step], legNum);
    }

    // Advance step
    gaitState.step++;

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

void Animation::Strafe() {
    static int idleCount = 0;
    static const int idleThreshold = 100;
    int liftHeight = 50;
    int resolution = 50;

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
        return;
    }

    // Ensure gait config is set
    if (gaitState.config.empty())
        gaitState.config = GetLegConfig(currentGait);

    // Calculate stride multiplier safely
    double strideMultiplier = 1.0;
    if (gaitState.config.size() > 1)
        strideMultiplier = 1.0 / (gaitState.config.size() - 1);

    // Generate trajectories at the start of each phase
    if (gaitState.step == 0) {
        auto swingGroup = gaitState.config[gaitState.phase];
        std::vector<int> stanceGroup;

        // Build stance group (all legs not in swing group)
        for (int idx = 0; idx < gaitState.config.size(); ++idx) {
            if (idx == gaitState.phase) continue;
            for (int legNum : gaitState.config[idx])
                stanceGroup.push_back(legNum);
        }

        // Clear sizes for all legs
        for (int i = 1; i <= MAX_LEGS; ++i) {
            gaitState.swingSizes[i] = 0;
            gaitState.stanceSizes[i] = 0;
        }

        // Generate swing trajectories
        for (int legNum : swingGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            Vector3 targetPos = cal.direction(controlPacket.joystick1X, controlPacket.joystick1Y, currentPos, legNum);
            int size = 0;
            cal.GenerateBezierTrajectory(
                gaitState.swingTrajectory[legNum].data(),
                size,
                currentPos,
                targetPos,
                liftHeight,
                resolution
            );
            gaitState.swingSizes[legNum] = size;
        }

        // Generate stance trajectories
        for (int legNum : stanceGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            Vector3 targetPos = cal.direction(controlPacket.joystick1X, controlPacket.joystick1Y, currentPos, legNum, true, strideMultiplier);
            int size = 0;
            cal.GenerateStraightTrajectory(
                gaitState.stanceTrajectory[legNum].data(),
                size,
                currentPos,
                targetPos,
                resolution
            );
            gaitState.stanceSizes[legNum] = size;
        }
    }

    // Move all legs for this step
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int swingSize = gaitState.swingSizes[legNum];
        int stanceSize = gaitState.stanceSizes[legNum];
        if (gaitState.step < swingSize)
            move.Position(gaitState.swingTrajectory[legNum][gaitState.step], legNum);
        if (gaitState.step < stanceSize)
            move.Position(gaitState.stanceTrajectory[legNum][gaitState.step], legNum);
    }

    // Advance step if not idle
    if (!stickIdle)
        gaitState.step++;

    // Phase transition
    if (gaitState.step > resolution) {
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size();
        gaitState.step = 0;
    }
}   
