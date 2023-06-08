// mapper.cpp : Defines the entry point for the application.
//

// STL
#ifndef _MSC_VER
    #include "ros/ros.h"
#endif
#include <iostream>

// Local classes
#include "mazeMap.h"

int main() {
    MazeMap maze(5, 3, 1);
    maze.printMap();

    // Move
    maze.currentPose.position += Position<double>{1.0, 1.0};
    std::cout << "\n\n\n"
              << std::endl;
    maze.printMap();

    // Add walls
    (*maze.currentTile()->down) = true;
    std::cout << "\n\n\n"
              << std::endl;
    maze.printMap();
    return 0;
}
