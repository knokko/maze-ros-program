#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

//           ^
//      y   /
//      ^  /<-
//      | /    \
//      |/ omega|
//      ------------> x
// (0,0)
///////////////////////

template<typename T>
struct Position {
        T x = 0;
        T y = 0;

        template<typename T>
        T& operator+=(const T& rhs) {
            this->x += rhs.x;
            this->y += rhs.y;
            return *this;
        }

        template<typename T>
        T& operator-=(const T& rhs) {
            this->x -= rhs.x;
            this->y -= rhs.y;
            return *this;
        }
};

template<typename T>
inline T operator+(T lhs, const T& rhs) {
    lhs += rhs;
    return lhs;
}
template<typename T>
inline T operator-(T lhs, const T& rhs) {
    lhs -= rhs;
    return lhs;
}

struct Pose {
        Position<double> position{0, 0};
        double           omega = 0;

        // Convience functions
        constexpr double x() const { return position.x; }
        constexpr double y() const { return position.y; }

        // Operators
        Pose operator+=(const Pose& other) {
            position += other.position;
            omega += other.omega;
        }

        Pose operator-=(const Pose& other) {
            position -= other.position;
            omega -= other.omega;
        }
};

/**
 * @brief Tile of the maze
 * @details Represents one K'nex square
 */
using Wall    = std::optional<bool>;
using WallPtr = std::shared_ptr<Wall>;
struct Tile {
        Tile(double size)
              : size(size){};

        // Tile parameters
        bool             visited = false;

        Position<double> pos;

        // Walls
        WallPtr      up;
        WallPtr      down;
        WallPtr      left;
        WallPtr      right;

        const double size;
};
using TilePtr = std::shared_ptr<Tile>;

class MazeMap {
    public:
        MazeMap(size_t width, size_t height, double tileSize, Position<double> initialPosition = {0, 0});

        // Location variables
        Pose                   currentPose;
        const double           tileSize;
        const Position<double> initialPosition;

        TilePtr                currentTile() const;
        Position<int>          currentTilePosition() const;

        void                   printMap() const;

    private:
        /**
         * @brief Map of the tiles, corresponding to the maze
         * @details The origin lays in the top left
         */
        std::vector<std::vector<TilePtr>> tileMap;

        /**
         * @brief
         * @details Walls are defined from top to bottom, the first row is the top of the tile map
         */
        std::vector<std::vector<WallPtr>> wallMap;
};