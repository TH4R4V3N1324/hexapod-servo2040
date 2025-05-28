#include "Animation.h"

//Returns leg configuration based on given gate
std::vector<std::vector<int>> Animation::GetLegConfig(Gait gait){
    switch (gait){
        case tripod:
            return {{1, 3, 5}, {2, 4, 6}};
            break;
        case ripple:
            return {{1, 6}, {3, 5}, {4, 2}};
            break;
        case wave:
            return {{6}, {5}, {4}, {3}, {2}, {1}};
            break;
        default:
            return {};
            break;
    }
}

//Changes to the next gait when called
void Animation::CycleGait(){
    currentGait = static_cast<Gait>((currentGait + 1) % 3);
}

//Changes to the next mode when called
void Animation::CycleMode(){
    currentMode = static_cast<Mode>((currentMode + 1) % 3);
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
    Shutdown();
    for(size_t i = 1; i <= 6; ++i){
        move.Position(startPos, i);
    }
}

void Animation::Strafe(){
    std::vector<std::vector<int>> config;
    int liftHeight = 30;
    int resolution = 50;
    
    switch (currentGait){
        case tripod: {
            gaitState.config = GetLegConfig(tripod);
            if (gaitState.step == 0){
                auto swingGroup = gaitState.config[gaitState.phase];
                auto stanceGroup = gaitState.config[1 - gaitState.phase];
                gaitState.swingTrajectory.clear();
                gaitState.stanceTrajectory.clear();

                for (int legNum : swingGroup) {
                    Vector3 currentPos = move.GetLegPosition(legNum);
                    Vector3 targetPos = cal.direction(currentPos, legNum);
                    gaitState.swingTrajectory[legNum] = cal.GenerateArcTrajectory(currentPos, targetPos, liftHeight, resolution);
                }

                for (int legNum : stanceGroup) {
                    Vector3 currentPos = move.GetLegPosition(legNum);
                    Vector3 targetPos = cal.direction(currentPos, legNum, true);
                    gaitState.stanceTrajectory[legNum] = cal.GenerateArcTrajectory(currentPos, targetPos, liftHeight, resolution, true);
                }
            }

            for (const auto& [legNum, trajectory] : gaitState.swingTrajectory) {
                move.Position(trajectory[gaitState.step], legNum);
            }

            for (const auto& [legNum, trajectory] : gaitState.stanceTrajectory) {
                move.Position(trajectory[gaitState.step], legNum);
            }

            gaitState.step++;
            if (gaitState.step > resolution) {
                gaitState.step = 0;
                gaitState.phase = 1 - gaitState.phase; // Switch phase
            }
            break;
        }
        case ripple:
            printf("RIPPLE");
            break;
        case wave:
            printf("WAVE");
            break;
        default:
            printf("INVALID GAIT");
            break;
    }
}