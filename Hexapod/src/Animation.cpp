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
        move.Deactivate(i);
    }
}

//move to home position for all legs
void Animation::Startup(){
    Shutdown();
    sleep_ms(5000);
    for(size_t i = 1; i <= 6; ++i){
        move.Position(startPos, i);
    }
}

void Animation::Strafe(){
    std::vector<std::vector<int>> config;
    int liftHeight = 20;
    int resolution = 15;
    double stepTime = 0.1;
    double maxVelocity = 20.0;
    
    switch (currentGait){
        case tripod: {
            config = GetLegConfig(tripod);
            
            auto performPhase = [&](const std::vector<int>& swingGroup, const std::vector<int>& stanceGroup) {
                std::map<int, std::vector<Vector3>> swingTrajectories;
                std::map<int, std::vector<Vector3>> stanceTrajectories;

                for (int legNum : swingGroup) {
                    Vector3 currentPos = move.GetLegPosition(legNum);
                    Vector3 targetPos = cal.direction(currentPos, stepTime, maxVelocity);
                    swingTrajectories[legNum] = cal.GenerateArcTrajectory(currentPos, targetPos, liftHeight, resolution);
                }

                for (int legNum : stanceGroup) {
                    Vector3 currentPos = move.GetLegPosition(legNum);
                    Vector3 targetPos = cal.direction(currentPos, stepTime, maxVelocity, true);
                    stanceTrajectories[legNum] = cal.GenerateStraightTrajectory(currentPos, targetPos, resolution);
                }

                for (int step = 0; step <= resolution; ++step) {
                    for (int legNum : swingGroup) {
                        move.Position(swingTrajectories[legNum][step], legNum);
                    }
                    for (int legNum : stanceGroup) {
                        move.Position(stanceTrajectories[legNum][step], legNum);
                    }
                    sleep_ms(20);
                }
                
                /*
                while (!move.AllLegsGrounded()) {
                    printf("legs not on ground");
                    sleep_ms(10);
               }*/ 
            };

            performPhase(config[0], config[1]);
            performPhase(config[1], config[0]);
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