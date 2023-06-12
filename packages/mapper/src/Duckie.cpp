#include "Duckie.h"


Duckie::Duckie() {
    int              mapWidth        = 5;
    int              mapHeight       = 5;
    double           tileSize        = 1;
    Position<double> initialPosition = {0.0, 0.0};

    // Create the maze
    maze = new MazeMap(mapWidth, mapHeight, tileSize, initialPosition);
}

Duckie::~Duckie() {
    delete maze;
}

void Duckie::run(std::stop_token stopToken) {
    initialize();

    while(!stopToken.stop_requested())
    {
        // Check sensor data

        // Calculate the next move

        // Instruct to drive
    }
}

bool Duckie::initialize() {
    currentState = std::make_unique<DebugState>();
    return true;
}
