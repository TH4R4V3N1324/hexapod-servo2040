#include "Animation.h"

//Returns leg configuration based on given gate
std::vector<std::vector<int>> Animation::GetLegConfig(Gait gait){
    switch (gait){
        case tripod:
            return {{1, 3, 5}, {2, 4, 6}};
            break;
        case ripple:
            return {{3, 6}, {2, 4}, {1, 5}};
            break;
        case wave:
            return {{3}, {2}, {1}, {4}, {5}, {6}};
            break;
        default:
            return {};
            break;
    }
}

//Changes to the next gait when called
void Animation::CycleGait(){
    pendingGait = static_cast<Gait>((currentGait + 1) % NumGaits);
    gaitChangeRequested = true;
}

//Changes to the next mode when called
void Animation::CycleMode(){
    currentMode = static_cast<Mode>((currentMode + 1) % NumModes);
}

//move to home, deactivate servos
void Animation::Shutdown(){
    for(size_t i = 1; i <= 6; ++i){
        move.Position(homePos, i);
        sleep_ms(1000);
        move.Deactivate(i);
    }
}

//move to home position for all legs
void Animation::Startup(){
    static bool initialized = false;

    if(!initialized){
        Shutdown();
        initialized = true;
    }
    
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
    if (gaitState.step == 0 && !trajectoryGenerated) {
        // Clear sizes for all legs
        for (int i = 1; i <= MAX_LEGS; ++i) {
            gaitState.swingSizes[i] = 0;
            gaitState.stanceSizes[i] = 0;
        }

        for (int index = 0; index < gaitState.config.size(); index++) {
            if (index == gaitState.phase) continue;
            for (int legNum : gaitState.config[index]) stanceGroup.push_back(legNum);
        }

        for (int legNum : swingGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            int size = 0;
            if (startPosition.find(legNum) == startPosition.end()) {
                printf("startPosition missing for legNum: %d\n", legNum);
            continue; // or handle error
            }
            cal.GenerateBezierTrajectory(gaitState.swingTrajectory[legNum].data(), size, currentPos, startPosition.at(legNum), liftHeight, resolution);
            gaitState.swingSizes[legNum] = size;
        }

        for (int legNum : stanceGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            for (int i = 0; i <= resolution; ++i) gaitState.stanceTrajectory[legNum][i] = currentPos;
            gaitState.stanceSizes[legNum] = resolution + 1;
        }
    }

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int swingSize = gaitState.swingSizes[legNum];
        int stanceSize = gaitState.stanceSizes[legNum];
        
        if (gaitState.step < swingSize) move.Position(gaitState.swingTrajectory[legNum][gaitState.step], legNum);
        if (gaitState.step < stanceSize) move.Position(gaitState.stanceTrajectory[legNum][gaitState.step], legNum);
    }

    gaitState.step++;
    if (gaitState.step > resolution) {
        counter++;
        gaitState.step = 0;
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size(); // Switch phase

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

    bool stickIdle = (std::abs(receivedData.LStickX) <= 10 && std::abs(receivedData.LStickY) <= 10);

    if (stickIdle) idleCount++; else idleCount = 0;

    if (idleCount > idleThreshold || gaitState.idleReturning) {
        if (!gaitState.idleReturning) {
            gaitState.idleReturning = true;
            gaitState.step = 0;
        }

        returnToStart();
        idleCount = 0;
        return;
    }

    if (gaitState.config.empty()) gaitState.config = GetLegConfig(currentGait);

    double strideMultiplier = 1.0 / (gaitState.config.size() - 1);

    if (gaitState.step == 0) {
        auto swingGroup = gaitState.config[gaitState.phase];
        std::vector<int> stanceGroup;

        // Clear sizes for all legs
        for (int i = 1; i <= MAX_LEGS; ++i) {
            gaitState.swingSizes[i] = 0;
            gaitState.stanceSizes[i] = 0;
        }

        for (int index = 0; index < gaitState.config.size(); index++) {
            if (index == gaitState.phase) continue;
            for (int legNum : gaitState.config[index]) stanceGroup.push_back(legNum);
        }

        for (int legNum : swingGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            Vector3 targetPos = cal.direction(currentPos, legNum);
            int size = 0;
            cal.GenerateBezierTrajectory(gaitState.swingTrajectory[legNum].data(), size, currentPos, targetPos, liftHeight, resolution);
            gaitState.swingSizes[legNum] = size;
        }

        for (int legNum : stanceGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            Vector3 targetPos = cal.direction(currentPos, legNum, true, strideMultiplier);
            int size = 0;
            cal.GenerateStraightTrajectory(gaitState.stanceTrajectory[legNum].data(), size, currentPos, targetPos, resolution);
            gaitState.stanceSizes[legNum] = size;
        }
    }

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int size = gaitState.swingSizes[legNum];
        if (gaitState.step < size) {
            move.Position(gaitState.swingTrajectory[legNum][gaitState.step], legNum);
        }
    }

    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int size = gaitState.stanceSizes[legNum];
        if (gaitState.step < size) {
            move.Position(gaitState.stanceTrajectory[legNum][gaitState.step], legNum);
        }
    }

    if (!stickIdle) gaitState.step++;

    if (gaitState.step > resolution) {
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size(); // Switch phase
        gaitState.step = 0;
    }
}   
