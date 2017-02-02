#include <cmath>
#include <list>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

//TODO mix going diagonally and inlinelly to make path shorter for robot
//TODO what if goal is on obstacle?
//TODO make class from this code
//TODO add proper visualization
//TODO make class responsible for friendly obstacles creation

#define WIDTH 50
#define HEIGHT 50
#define INLINE_COST 10
#define DIAGONAL_COST 14
#define DELAY 10000

struct Point{
    int x, y;

    Point(){}

    Point(int x, int y){
        this->x = x;
        this->y = y;
    }

    bool operator == (Point &p){
        return (this->x == p.x && this->y == p.y);
    }

    bool operator != (Point &p){
        return (this->x != p.x || this->y != p.y);
    }
};

struct Node{
    bool isFree;
    bool wasClosed;
    Node* parent;
    std::list<Node*> inlineNeighbours;
    std::list<Node*> diagonalNeighbours;
    Point point;
    int costSoFar;
    int heuristic;

    Node(){
        this->wasClosed = false;
        this->parent = NULL;
        this->costSoFar = INT_MAX;
        this->heuristic = INT_MAX;
    }

    Node(bool isFree, Point point) : Node(){
        this->isFree = isFree;
        this->point = point;
    }
};

//visualization
void printBoard(Node (&board)[WIDTH][HEIGHT]){
    for(int i=0; i<WIDTH; i++){
        for(int j=0; j<HEIGHT; j++){
            if(board[i][j].wasClosed)
                printf("x");
            else if(board[i][j].isFree)
                printf(".");
            else
                printf("0");
        }

        printf("\n");
    }
}
// /visualization

bool isInBounds(Point p){
    return (p.x >= 0 && p.x < WIDTH && p.y >= 0 && p.y < HEIGHT);
}

int getHeuristic(Point &start, Point &goal){
    int xDiff = std::abs(start.x - goal.x);
    int yDiff = std::abs(start.y - goal.y);
    int minDiff = std::min(xDiff, yDiff);
    int maxDiff = std::max(xDiff, yDiff);
    return DIAGONAL_COST*minDiff + INLINE_COST*(maxDiff-minDiff);
}

std::list<Point> getPath(Node (&board)[WIDTH][HEIGHT], Point &start, Point &goal){
    if(!isInBounds(start) || !isInBounds(goal))
        return std::list<Point>();

    std::list<Node*> openNodes;
    Node* startNode = &board[start.x][start.y];
    startNode->costSoFar = 0;
    startNode->heuristic = getHeuristic(startNode->point, goal);
    openNodes.push_back(startNode);
    Node* currNode;
    //printf("starting A* loop...\n");
    do{
        currNode = openNodes.front();
        openNodes.pop_front();

        std::system("clear");
        printBoard(board);
        usleep(DELAY);
        //printf("current node: (%d, %d)\n", currNode->point.x, currNode->point.y);

        currNode->wasClosed = true;

        while(!openNodes.empty() && openNodes.front()->wasClosed)
            openNodes.pop_front();

        for(std::list<Node*>::iterator it = currNode->inlineNeighbours.begin(); it != currNode->inlineNeighbours.end(); it++){
            if(!(*it)->isFree || (*it)->wasClosed)
                continue;

            //printf("  neighbour: (%d, %d) ", (*it)->point.x, (*it)->point.y);
            (*it)->heuristic = getHeuristic((*it)->point, goal);
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

                    if((*it)->costSoFar+(*it)->heuristic < (*openIt)->costSoFar+(*openIt)->heuristic){
                        openNodes.insert(openIt, (*it));
                        break;
                    }
                }
            }

            //printf("\n");
        }

        for(std::list<Node*>::iterator it = currNode->diagonalNeighbours.begin(); it != currNode->diagonalNeighbours.end(); it++){
            if(!(*it)->isFree || (*it)->wasClosed)
                continue;

            (*it)->heuristic = getHeuristic((*it)->point, goal);
            int potentialCostSoFar = currNode->costSoFar + DIAGONAL_COST;

            if(potentialCostSoFar < (*it)->costSoFar){
                (*it)->costSoFar = potentialCostSoFar;
                (*it)->parent = currNode;

                for(std::list<Node*>::iterator openIt = openNodes.begin();; openIt++){
                    if(openIt == openNodes.end()){
                        openNodes.push_back((*it));
                        break;
                    }

                    if((*it)->costSoFar+(*it)->heuristic < (*openIt)->costSoFar+(*openIt)->heuristic){
                        openNodes.insert(openIt, (*it));
                        break;
                    }
                }
            }
        }

        if(openNodes.empty())
            return std::list<Point>();

    }
    while(currNode->point != goal);

    std::list<Point> path;
    while(currNode != NULL){
        path.push_front(currNode->point);
        currNode = currNode->parent;
    }

    return path;

}

void initializeNeighbours(Node (&board)[WIDTH][HEIGHT]){
    for(int i=0; i<WIDTH; i++){
        for(int j=0; j<HEIGHT; j++){
            if(j > 0)
                board[i][j].inlineNeighbours.push_back(&board[i][j-1]);
            if(i+1 < WIDTH)
                board[i][j].inlineNeighbours.push_back(&board[i+1][j]);
            if(j+1 < HEIGHT)
                board[i][j].inlineNeighbours.push_back(&board[i][j+1]);
            if(i > 0)
                board[i][j].inlineNeighbours.push_back(&board[i-1][j]);

            if(i > 0 && j > 0)
                board[i][j].diagonalNeighbours.push_back(&board[i-1][j-1]);
            if(i+1 < WIDTH && j > 0)
                board[i][j].diagonalNeighbours.push_back(&board[i+1][j-1]);
            if(i+1 < WIDTH && j+1 < HEIGHT)
                board[i][j].diagonalNeighbours.push_back(&board[i+1][j+1]);
            if(i > 0 && j+1 < HEIGHT)
                board[i][j].diagonalNeighbours.push_back(&board[i-1][j+1]);
        }
    }
}

void resetBoard(Node (&board)[WIDTH][HEIGHT]){
    for(int i=0; i<WIDTH; i++){
        for(int j=0; j<HEIGHT; j++){
            board[i][j].parent = NULL;
            board[i][j].costSoFar = INT_MAX;
        }
    }
}

int main(int argc, char **argv){
    usleep(DELAY*200);
    bool obstacles[WIDTH][HEIGHT]; //TODO wczytywanie
    for(int i=1; i<10; i++){
        obstacles[i][i] = true;
        obstacles[i+1][i] = true;
        //obstacles[i-1][i-1] = true;
    }

    printf("obstacles loaded\n");
    Node board[WIDTH][HEIGHT];
    for(int i=0; i<WIDTH; i++){
        for(int j=0; j<HEIGHT; j++){
            Point p = Point(i, j);
            board[i][j] = Node(!obstacles[i][j], p);
        }
    }

    initializeNeighbours(board);
    printf("board initialized\n");

    //loop TODO
    Point start = Point(10, 0); //TODO wczytywanie
    Point goal = Point (0, 10); //TODO wczytywanie
    printf("start and goal points loaded\n");

    std::list<Point> path = getPath(board, start, goal); //TODO
    printf("path calculated\n");
    /*for(std::list<Point>::iterator it = path.begin(); it != path.end(); it++)
        printf("(%d, %d)\n", it->x, it->y);*/

    if(path.empty())
        printf("goal is not accessible");
    // /loop
    return 0;
}
