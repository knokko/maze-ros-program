#pragma once

// STL
#include <memory>
#include <thread>

// Includes
#include <MazeMap.h>
#include <States.h>

class Duckie {
        friend AbstractState;

    public:
        Duckie();
        ~Duckie();

        void run(std::stop_token stopToken);


    private:
        bool initialize();

        MazeMap*    maze = nullptr;
        std::unique_ptr<AbstractState> currentState;
};
