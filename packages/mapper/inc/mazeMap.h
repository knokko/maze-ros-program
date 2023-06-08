#pragma once
#include <memory>

struct Tile;

using NeighbourTile = std::shared_ptr<const Tile>;

struct Pos {
        int x = 0;
        int y = 0;
};

struct Tile {
        Tile(Pos position) : pos(position) {}

        bool visited = false;

        Pos pos;

        // Neighbours
        NeighbourTile up    = nullptr;
        NeighbourTile down  = nullptr;
        NeighbourTile left  = nullptr;
        NeighbourTile right = nullptr;
};

class MazeMap {
    public:
        MazeMap(Pos initialPosition)
              : currentTile(std::make_shared<Tile>(initialPosition)){};
        ~MazeMap();

        std::shared_ptr<Tile> currentTile;

    private:
};