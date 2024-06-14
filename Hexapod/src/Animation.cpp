#include "Animation.h"

//Returns leg configuration based on given gate
std::vector<std::vector<int>> Animation::GetLegConfig(Gait gait){
    switch (gait){
        case tripod:
            return {{1, 3, 5}, {2, 4, 6}};
        case ripple:
            return {{1, 6}, {3, 5}, {4, 2}};
        case wave:
            return {{6}, {5}, {4}, {3}, {2}, {1}};
        default:
            return {};
    }
};

//move to home, deactivate servos
void Animation::Shutdown(){
    std::vector<double> homePos {110, 110, 0};
    Move move;

    for(size_t i = 0; i < 7; ++i){
        move.Position(homePos, i);
        move.Deactivate(i);
    };
};

//move to home position for all legs
void Animation::Startup(){
    void Shutdown();
    sleep_ms(5000);

    std::vector<double> startPos {120, 120, -100};
    Move move;
    for(size_t i = 0; i < 7; ++i){
        move.Position(startPos, i);
    }; 
};

//walking animation consistong of arc and line
void Animation::Walk(std::vector<double> start, std::vector<double> end){
    
};

//rotate animation to turn the hexapod in a given direction
void Animation::Rotate(){
    //code
}
