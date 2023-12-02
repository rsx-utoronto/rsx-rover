#ifndef Map_h
#define Map_h

#include "globals.h"

class Map {
    public:
      int Map::get_map(int row, int col);

    private:
      int map[mapSize][mapSize];
}; 

#endif