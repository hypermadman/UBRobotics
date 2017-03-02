struct Node{
    bool isObstacle;
    bool wasClosed;
    Node* parent;
    std::list<Node*> inlineNeighbours;
    std::list<Node*> diagonalNeighbours;
    Vector2D position;
    int costSoFar;
    int heuristic;

    Node(){
        this->isObstacle = false;
        this->wasClosed = false;
        this->parent = NULL;
        this->costSoFar = INT_MAX;
        this->heuristic = INT_MAX;
    }

    Node(Vector2D position) : Node(){
        this->position = position;
    }
};
