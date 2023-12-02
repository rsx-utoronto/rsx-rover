#include "Node.h"
#include "Map.h"
#include "Node.h"
#include <vector>
#include <cmath>

using namespace std;

vector<Node*> aStar(Map& map, Node* start, Node* end);
float heuristic(const Node* a, const Node* b);
bool isValid(int x, int y, Map& map);

struct Compare {
    bool operator()(const Node* a, const Node* b) const {
        return a->f > b->f;
    }
};