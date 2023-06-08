#include <mazeMap.h>

MazeMap::MazeMap(size_t width, size_t height, double tileSize, Position<double> initialPosition)
      : tileSize(tileSize), currentPose({initialPosition, 0}) {
    // Initialize Walls
    for(size_t y = 0; y < height + 1; y++)
    {
        wallMap.push_back(std::vector<WallPtr>(width + 1, std::make_shared<Wall>()));
    }

    // Initialize Tiles
    for(size_t y = 0; y < height; y++)
    {
        auto tileRow = std::vector<TilePtr>();
        for(size_t x = 0; x < width; x++)
        {
            auto tile = std::make_shared<Tile>(tileSize);

            // First row needs to have the top walls additionally
            if(y == 0)
            {
                tile->up = wallMap[y][x];
            }
            else
            {
                tile->up = tileMap.back()[x]->down;
            }

            // The first column needs to have the left wall extra
            if(x == 0)
            {
                tile->left = wallMap[y][x];
            }
            else
            {
                tile->left = tileRow.back()->right;
            }

            // The rest of the walls are the same
            tile->down  = wallMap[y][x + 1];
            tile->right = wallMap[y + 1][x];

            tileRow.push_back(tile);
        }
        tileMap.push_back(tileRow);
    }
}

TilePtr MazeMap::currentTile() const {
    const auto pos = currentTilePosition();
    return tileMap[pos.y][pos.x];
}

Position<int> MazeMap::currentTilePosition() const {
    const auto pos = currentPose.position;

    // Use modulo to get the tile position
    int tileX = static_cast<int>(std::floor(pos.x / tileSize));
    int tileY = static_cast<int>(std::floor(pos.y / tileSize));
    return {tileX, tileY};
}

void MazeMap::printMap() const {
    // Every tile is printed as a 3x3 square with the walls.
    // The middle of the square is a dot
    for(size_t y = 0; y < tileMap.size(); y++)
    {
        // Print the top walls
        for(size_t x = 0; x < tileMap[y].size(); x++)
        {
            auto tile = tileMap[y][x];
            if(tile->up->has_value())
            {
                std::cout << (tile->up->value() ? " - " : ".  .");
            }
            else
            {
                std::cout << "   ";
            }
        }
        std::cout << std::endl;
        // Print the left walls and the dots
        for(size_t x = 0; x < tileMap[y].size(); x++)
        {
            auto tile = tileMap[y][x];
            // Left wall
            if(tile->left->has_value())
            {
                std::cout << (tile->left->value() ? "|" : ".");
            }
            else
            {
                std::cout << " ";
            }

            // Middle dot
            if(currentTilePosition().x == x && currentTilePosition().y == y)
            {
                std::cout << "X";
            }
            else
            {
                std::cout << "o";
            }

            // Right wall
            if(tile->right->has_value())
            {
                std::cout << (tile->right->value() ? "|" : ".");
            }
            else
            {
                std::cout << " ";
            }
        }
        std::cout << std::endl;

        // Print the bottom walls
        for(size_t x = 0; x < tileMap[y].size(); x++)
        {
            auto tile = tileMap[y][x];
            if(tile->down->has_value())
            {
                std::cout << tile->down->value() ? " - " : ".  .";
            }
            else
            {
                std::cout << "   ";
            }
        }
        std::cout << std::endl;
    }
}
