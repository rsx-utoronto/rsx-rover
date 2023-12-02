#include "Map.h"
#include <iostream>

int main () {
    Map map;

    for (int i = 0; i < mapSize; i++) {
        for (int j = 0; j < mapSize; j++) {
            std::cout << map.get_map(i, j);
        }
        std::cout << "\n";
    }

    return 0;

}