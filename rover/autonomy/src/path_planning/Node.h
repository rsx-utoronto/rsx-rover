#ifndef Node_h
#define Node_h

struct Node {
    int x, y;           
    float g;            
    float h;            
    float f;           
    Node* parent;  
};

#endif