#include "Map.h"
#include <cstdlib>

Map::Map() {
    for (int i = 0; i < mapSize; i++) {
        for (int j = 0; j < mapSize; j++) {
            int num = rand() % 10;
            if (num < 3) {
                map[i][j] = X;
            } else {
                map[i][j] = O;
            }
        }
    }
}

int Map::get_map(int row, int col) {
    return map[row][col];
}