#include <mazeMap.h>

MazeMap::MazeMap(size_t width, size_t height, double tileSize, Position<double> initialPosition)
      : tileSize(tileSize), currentPose({initialPosition, 0}) {
    // Initialize Walls
    for(size_t y = 0; y < height * 2 + 1; y++)
    {
        std::vector<WallPtr> row;
        for(size_t x = 0; x < width * 2 + (y % 2); x++)
        {
            row.push_back(std::make_shared<Wall>());
        }
        wallMap.push_back(row);
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
                tile->up = wallMap[0][x];
            }
            else
            {
                tile->up = tileMap.back()[x]->down;
            }

            // The first column needs to have the left wall extra
            if(x == 0)
            {
                tile->left = wallMap[2 * y + 1][0];
            }
            else
            {
                tile->left = tileRow.back()->right;
            }

            // The rest of the walls are the same
            tile->right = wallMap[2 * y + 1][2 * (x + 1)];
            tile->down  = wallMap[2 * (y + 1)][2 * x];

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

    // Print the rest
    for(size_t y = 0; y < wallMap.size(); y++)
    {
        for(size_t x = 0; x < wallMap[y].size(); x++)
        {
            const auto wall = wallMap.at(y).at(x).get();
            // Odd rows contain only walls and the even rows have dots and walls
            if(y % 2)
            {
                // This is a row with walls and points
                // Again, we can split by parity
                if(x % 2)
                {
                    // Print dot
                    if((currentTilePosition().x * 2 + 1) == x && (currentTilePosition().y * 2 + 1) == y)
                    {
                        std::cout << "X";
                    }
                    else
                    {
                        std::cout << "o";
                    }
                }
                else
                {
                    // Print wall
                    if(wall->has_value())
                    {
                        std::cout << (wall->value() ? "|" : " ");
                    }
                    else
                    {
                        std::cout << ":";
                    }
                }
            }
            // This is a row with only walls
            else
            {
                // Print offset for first column
                if(x == 0)
                {
                    std::cout << " ";
                }

                if(wall->has_value())
                {
                    std::cout << (wall->value() ? "-" : " ");
                }
                else
                {
                    std::cout << ".";
                }
            }
        }
        std::cout << std::endl;
    }
}
