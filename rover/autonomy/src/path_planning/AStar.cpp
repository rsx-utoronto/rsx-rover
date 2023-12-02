#include "AStar.h"
#include <queue>
#include <iostream>


vector<Node*> aStar(Map& map, Node* start, Node* end){

    if(!isValid(start->x, start->y, map)||!isValid(end->x, end->y, map)){
        vector<Node*> path;
        return path;
    }

    priority_queue<Node*, vector<Node*>, Compare> openList;

    openList.push(start);

    float closedList[mapSize][mapSize];

    for (int i = 0; i < mapSize; i++) {
    for (int j = 0; j < mapSize; j++) {
        closedList[i][j] = -1;
    }
}

    while(!openList.empty()){
        Node* current = openList.top();
        openList.pop();
        //printing the node being tested
        //cout<<"Testing x:"<<current->x<<", y:"<<current->y<<'\n';

        closedList[current->x][current->y] = current->g;

        //path found
        if (current->x == end->x && current->y == end->y){
            vector<Node*> path;
            while(current){
                path.push_back(current);
                current = current->parent;
            }

            reverse(path.begin(), path.end());
            return path;
        }

        int directions[8][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        for (int i = 0; i<8; i++){

            int new_x = current->x + directions[i][0];
            int new_y = current->y + directions[i][1];

            if(!isValid(new_x, new_y, map))
                continue;
            
            float new_g = current->g + sqrt(directions[i][0]*directions[i][0] + directions[i][1]*directions[i][1]);

            if (closedList[new_x][new_y] == -1 || new_g < closedList[new_x][new_y]){
                Node* neighbor = new Node();
                neighbor->x = new_x;
                neighbor->y = new_y;
                neighbor->g = new_g;
                neighbor->h = heuristic(neighbor, end);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                closedList[new_x][new_y] = new_g;

                openList.push(neighbor);
            }

        }
        
    }
    vector<Node*> path;
    return path; 

}
float heuristic(const Node* a, const Node* b){
    return sqrt(abs(a->x - b->x)*abs(a->x - b->x) + abs(a->y - b->y)*abs(a->y - b->y));
}
bool isValid(int x, int y, Map& map){
     return (x >= 0 && y >= 0 && x < mapSize && y < mapSize && map.get_map(x, y) != X);
}