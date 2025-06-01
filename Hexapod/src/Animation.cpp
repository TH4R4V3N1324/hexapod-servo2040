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
    currentGait = static_cast<Gait>((currentGait + 1) % NumGaits);
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
        Vector3 pos = startPos;
        if(i == 1 || i == 6){
            double angleOffset = -15.0 * M_PI / 180.0; // -15 degrees in radians
            double x = startPos.x * cos(angleOffset) - startPos.y * sin(angleOffset);
            double y = startPos.x * sin(angleOffset) + startPos.y * cos(angleOffset);
            pos.x = x;
            pos.y = y;
        }
        else if(i == 3 || i == 4){
            double angleOffset = 15.0 * M_PI / 180.0; // +15 degrees in radians
            double x = startPos.x * cos(angleOffset) - startPos.y * sin(angleOffset);
            double y = startPos.x * sin(angleOffset) + startPos.y * cos(angleOffset);
            pos.x = x;
            pos.y = y;
        }
        move.Position(pos, i);
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

void Animation::returnToStart(){
    static int counter = 0;
    std::vector<std::vector<int>> config;
    gaitState.config = GetLegConfig(tripod);
    int liftHeight = 50;
    int resolution = 50;

    auto swingGroup = gaitState.config[gaitState.phase];
    std::vector<int> stanceGroup;
    if (gaitState.step == 0){
        gaitState.swingTrajectory.clear();
        gaitState.stanceTrajectory.clear();
        for (int index = 0; index < gaitState.config.size(); index++){
            if(index == gaitState.phase){continue;}
            for(int legNum : gaitState.config[index]) {
                stanceGroup.push_back(legNum);
            }
        }

        for (int legNum : swingGroup){
            Vector3 pos = startPos;
            if(legNum == 1 || legNum == 6){
                double angleOffset = -15.0 * M_PI / 180.0; // -15 degrees in radians
                double x = startPos.x * cos(angleOffset) - startPos.y * sin(angleOffset);
                double y = startPos.x * sin(angleOffset) + startPos.y * cos(angleOffset);
                pos.x = x;
                pos.y = y;
            }
            else if(legNum == 3 || legNum == 4){
                double angleOffset = 15.0 * M_PI / 180.0; // +15 degrees in radians
                double x = startPos.x * cos(angleOffset) - startPos.y * sin(angleOffset);
                double y = startPos.x * sin(angleOffset) + startPos.y * cos(angleOffset);
                pos.x = x;
                pos.y = y;
            }
            gaitState.swingTrajectory[legNum] = cal.GenerateArcTrajectory(move.GetLegPosition(legNum), pos, liftHeight, resolution);
        }

        for (int legNum : stanceGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            gaitState.stanceTrajectory[legNum].resize(resolution + 1, currentPos); //= std::vector<Vector3>(resolution + 1, currentPos);
        }
    }

    for (const auto& [legNum, trajectory] : gaitState.swingTrajectory) {
        if (gaitState.step < trajectory.size()) {
            move.Position(trajectory[gaitState.step], legNum);
        }
    }

    for (const auto& [legNum, trajectory] : gaitState.stanceTrajectory) {
        if (gaitState.step < trajectory.size()) {
            move.Position(trajectory[gaitState.step], legNum);
        }
    }

    counter++;
    gaitState.step ++;
    if (gaitState.step > resolution) {
        gaitState.step = 0;
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size(); // Switch phase

        if (counter > gaitState.config.size()){
        gaitState.idleReturning = false;
        }
    }
    
}

void Animation::Strafe(){
    static int idleCount = 0;
    static const int idleThreshold = 100;

    bool stickIdle = (std::abs(receivedData.LStickX) <= 10 && std::abs(receivedData.LStickY) <= 10);

    if (stickIdle) {idleCount++;} else {idleCount = 0;}
    
    if (idleCount > idleThreshold || gaitState.idleReturning) {
        if (!gaitState.idleReturning) {gaitState.idleReturning = true;}
        if (idleCount > idleThreshold) {gaitState.step = 0;}

        returnToStart();

        idleCount = 0; // Reset idle count after processing

        return;
    }
    
    std::vector<std::vector<int>> config;
    int liftHeight = 50;
    int resolution = 50;

    if (gaitState.config.empty() || gaitState.config != GetLegConfig(currentGait)) {
        gaitState.config = GetLegConfig(currentGait);
    }

    printf("Current Gait: %d, Current Mode: %d\n", currentGait, currentMode);

    double configSize = gaitState.config.size();
    double strideMultiplier = 1 / (configSize - 1);

    if (gaitState.step == 0){
    auto swingGroup = gaitState.config[gaitState.phase];
    std::vector<int> stanceGroup;

        for (int index = 0; index < gaitState.config.size(); index++){
            if(index == gaitState.phase){continue;}
            for(int legNum : gaitState.config[index]) {
                stanceGroup.push_back(legNum);
            }
        }

        gaitState.swingTrajectory.clear();
        gaitState.stanceTrajectory.clear();

        for (int legNum : swingGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            Vector3 targetPos = cal.direction(currentPos, legNum);
            gaitState.swingTrajectory[legNum] = cal.GenerateArcTrajectory(currentPos, targetPos, liftHeight, resolution);
        }

        for (int legNum : stanceGroup) {
            Vector3 currentPos = move.GetLegPosition(legNum);
            Vector3 targetPos = cal.direction(currentPos, legNum, true, strideMultiplier);
            gaitState.stanceTrajectory[legNum] = cal.GenerateStraightTrajectory(currentPos, targetPos, resolution);
        }
    }

    for (const auto& [legNum, trajectory] : gaitState.swingTrajectory) {
        if (gaitState.step < trajectory.size()) {
            move.Position(trajectory[gaitState.step], legNum);
        }
    }

    for (const auto& [legNum, trajectory] : gaitState.stanceTrajectory) {
        if (gaitState.step < trajectory.size()) {
            move.Position(trajectory[gaitState.step], legNum);
        }
    }

    if (!stickIdle) {gaitState.step++;}

    if (gaitState.step > resolution) {
        gaitState.step = 0;
        gaitState.phase = (gaitState.phase + 1) % gaitState.config.size(); // Switch phase
    }
}   
