#include <cmath>
#include <list>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

//TODO mix going diagonally and inlinelly to make path shorter for robot
//TODO what if goal is on obstacle?
//TODO add proper visualization
//TODO make class responsible for friendly obstacles creation
//TODO make a board struct and add getters and setters

#define WIDTH 250
#define HEIGHT 75
#define INLINE_COST 10
#define DIAGONAL_COST 14
#define DELAY 10000

struct Vector2D{
    int x, y;

    Vector2D(){}

    Vector2D(int x, int y){
        this->x = x;
        this->y = y;
    }

    bool operator == (Vector2D &v){
        return (this->x == v.x && this->y == v.y);
    }

    bool operator != (Vector2D &v){
        return (this->x != v.x || this->y != v.y);
    }
};

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

//visualization
void printBoard(Node** board, Vector2D goal){
    for(int i=0; i<HEIGHT; i++){
        for(int j=0; j<WIDTH; j++){
            if(Vector2D(j, i) == goal)
                printf("!");
            else if(board[j][i].wasClosed)
                printf("x");
            else if(board[j][i].isObstacle)
                printf("0");
            else
                printf(".");
        }

        printf("\n");
    }
}
// /visualization

class PathFinder{
     public:
         PathFinder(Vector2D* obstacles, int obstaclesNumber, Vector2D _boardSize): boardSize(_boardSize){ //TODO extract init methods
            initializeBoard();
            initializeObstacles(obstacles, obstaclesNumber);
            initializeNeighbours();
        }

        std::list<Vector2D> getPath(Vector2D &start, Vector2D &goal){
            if(!isInBounds(start) || !isInBounds(goal))
                return std::list<Vector2D>();

            std::list<Node*> openNodes;
            Node* startNode = &board[start.x][start.y];
            startNode->costSoFar = 0;
            startNode->heuristic = getHeuristic(startNode->position, goal);
            openNodes.push_back(startNode);
            Node* currNode;
            //printf("starting A* loop...\n");
            do{
                currNode = openNodes.front();
                openNodes.pop_front();

                //usleep(DELAY);
                std::system("clear");
                printBoard(board, goal);

                //printf("current node: (%d, %d)\n", currNode->position.x, currNode->position.y);

                currNode->wasClosed = true;

                while(!openNodes.empty() && openNodes.front()->wasClosed)
                    openNodes.pop_front();

                for(std::list<Node*>::iterator it = currNode->inlineNeighbours.begin(); it != currNode->inlineNeighbours.end(); it++){
                    if((*it)->isObstacle || (*it)->wasClosed)
                        continue;

                    //printf("  neighbour: (%d, %d) ", (*it)->position.x, (*it)->position.y);
                    (*it)->heuristic = getHeuristic((*it)->position, goal);
                    int potentialCostSoFar = currNode->costSoFar + INLINE_COST;
                    //printf("potentialCost: %d, costSoFar: %d", potentialCostSoFar, (*it)->costSoFar);

                    if(potentialCostSoFar < (*it)->costSoFar){
                        (*it)->costSoFar = potentialCostSoFar;
                        (*it)->parent = currNode;

                        for(std::list<Node*>::iterator openIt = openNodes.begin();; openIt++){
                            if(openIt == openNodes.end()){
                                openNodes.push_back((*it));
                                break;
                            }

                            if((*it)->costSoFar+(*it)->heuristic <= (*openIt)->costSoFar+(*openIt)->heuristic){
                                openNodes.insert(openIt, (*it));
                                break;
                            }
                        }
                    }

                    //printf("\n");
                }

                for(std::list<Node*>::iterator it = currNode->diagonalNeighbours.begin(); it != currNode->diagonalNeighbours.end(); it++){
                    if((*it)->isObstacle || (*it)->wasClosed)
                        continue;

                    (*it)->heuristic = getHeuristic((*it)->position, goal);
                    int potentialCostSoFar = currNode->costSoFar + DIAGONAL_COST;

                    if(potentialCostSoFar < (*it)->costSoFar){
                        (*it)->costSoFar = potentialCostSoFar;
                        (*it)->parent = currNode;

                        for(std::list<Node*>::iterator openIt = openNodes.begin();; openIt++){
                            if(openIt == openNodes.end()){
                                openNodes.push_back((*it));
                                break;
                            }

                            if((*it)->costSoFar+(*it)->heuristic <= (*openIt)->costSoFar+(*openIt)->heuristic){
                                openNodes.insert(openIt, (*it));
                                break;
                            }
                        }
                    }
                }

                if(openNodes.empty())
                    return std::list<Vector2D>();

            }
            while(currNode->position != goal);

            std::list<Vector2D> path;
            while(currNode != NULL){
                path.push_front(currNode->position);
                currNode = currNode->parent;
            }

            return path;

        }

    private:
        Node** board;
        Vector2D boardSize;

        void initializeBoard(){
            board = new Node*[boardSize.x];
                for(int i=0; i<boardSize.x; i++){
                    board[i] = new Node[boardSize.y];

                    for(int j=0; j<boardSize.y; j++)
                        board[i][j] = Node(Vector2D(i, j));
                }
        }

        void initializeObstacles(Vector2D* obstacles, int obstaclesNumber){
             for(int i=0; i<obstaclesNumber; i++)
                board[obstacles[i].x][obstacles[i].y].isObstacle = true;
        }

        void initializeNeighbours(){
            for(int i=0; i<boardSize.x; i++){
                for(int j=0; j<boardSize.y; j++){
                    if(j > 0)
                        board[i][j].inlineNeighbours.push_back(&board[i][j-1]);
                    if(i+1 < boardSize.x)
                        board[i][j].inlineNeighbours.push_back(&board[i+1][j]);
                    if(j+1 < boardSize.y)
                        board[i][j].inlineNeighbours.push_back(&board[i][j+1]);
                    if(i > 0)
                        board[i][j].inlineNeighbours.push_back(&board[i-1][j]);

                    if(i > 0 && j > 0)
                        board[i][j].diagonalNeighbours.push_back(&board[i-1][j-1]);
                    if(i+1 < boardSize.x && j > 0)
                        board[i][j].diagonalNeighbours.push_back(&board[i+1][j-1]);
                    if(i+1 < boardSize.x && j+1 < boardSize.y)
                        board[i][j].diagonalNeighbours.push_back(&board[i+1][j+1]);
                    if(i > 0 && j+1 < boardSize.y)
                        board[i][j].diagonalNeighbours.push_back(&board[i-1][j+1]);
                }
            }
        }

        bool isInBounds(Vector2D p){
            return (p.x >= 0 && p.x < boardSize.x && p.y >= 0 && p.y < boardSize.y);
        }

        int getHeuristic(Vector2D &start, Vector2D &goal){
            int xDiff = std::abs(start.x - goal.x);
            int yDiff = std::abs(start.y - goal.y);
            int minDiff = std::min(xDiff, yDiff);
            int maxDiff = std::max(xDiff, yDiff);
            return (DIAGONAL_COST)*minDiff + (INLINE_COST)*(maxDiff-minDiff);
        }

       void resetBoard(Node (&board)[WIDTH][HEIGHT]){
            for(int i=0; i<boardSize.x; i++){
                for(int j=0; j<boardSize.y; j++){
                    board[i][j].parent = NULL;
                    board[i][j].costSoFar = INT_MAX;
                    board[i][j].heuristic = INT_MAX;
                    board[i][j].wasClosed = false;
                }
            }
        }
};

int main(int argc, char **argv){
    int obstaclesNumber = 18;
    Vector2D obstacles[18]; //TODO wczytywanie
    for(int i=0; i<10; i++){
        obstacles[i] = Vector2D(i+1, i+1);
        obstacles[i + obstaclesNumber/2] = Vector2D(i+2, i+1);
        //obstacles[i-1][i-1] = true;
    }

    printf("obstacles loaded\n");
    //PathFinder(obstacles) TODO

    PathFinder pathFinder = PathFinder(obstacles, obstaclesNumber, Vector2D(WIDTH, HEIGHT));
    printf("PathFinder initialized\n");

    usleep(DELAY*500);
    //loop TODO
    Vector2D start = Vector2D(200, 70); //TODO wczytywanie
    Vector2D goal = Vector2D (0, 0); //TODO wczytywanie
    printf("start and goal positions loaded\n");

    std::list<Vector2D> path = pathFinder.getPath(start, goal); //TODO
    printf("path calculated\n");

    if(path.empty())
        printf("goal is not accessible");
    // /loop
    return 0;
}

