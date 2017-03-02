#include <list>
#include <climits>
#include <cstdio>

//TODO mix going diagonally and inlinelly to make path shorter for robot
//TODO make class responsible for friendly obstacles creation

#define VISUALIZE 1 //0 for no visualization, 1 for visualization
#define WIDTH 250 //set width of the board
#define HEIGHT 75 //set height of the board

#define INLINE_COST 10 //don't change it, unless you know what you are doing
#define DIAGONAL_COST 14 //don't change it, unless you know what you are doing

#include "Vector2D.cpp"
#include "Node.cpp"
#include "Board.cpp"
#ifdef VISUALIZE
    #include "Visualizer.cpp"
#endif


class PathFinder{
     public:
        PathFinder(Vector2D* obstacles, int obstaclesNumber, Vector2D boardSize)
        : board(obstacles, obstaclesNumber, boardSize) {}

        std::list<Vector2D> getPath(Vector2D &start, Vector2D &goal){
            if(!board.isInBounds(start) || !board.isInBounds(goal))
                return std::list<Vector2D>();

            std::list<Node*> openNodes;
            Node* startNode = board.getNodePtr(start);
            startNode->costSoFar = 0;
            startNode->heuristic = getHeuristic(startNode->position, goal);
            openNodes.push_back(startNode);
            Node* currNode;
            do{
                currNode = openNodes.front();
                openNodes.pop_front();

                if(VISUALIZE)
                   Visualizer::visualize(board, goal);

                currNode->wasClosed = true;

                while(!openNodes.empty() && openNodes.front()->wasClosed)
                    openNodes.pop_front();

                for(std::list<Node*>::iterator it = currNode->inlineNeighbours.begin(); it != currNode->inlineNeighbours.end(); it++){
                    if((*it)->isObstacle || (*it)->wasClosed)
                        continue;

                    (*it)->heuristic = getHeuristic((*it)->position, goal);
                    int potentialCostSoFar = currNode->costSoFar + INLINE_COST;

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
        Board board;

        int getHeuristic(Vector2D &start, Vector2D &goal){
            int xDiff = std::abs(start.x - goal.x);
            int yDiff = std::abs(start.y - goal.y);
            int minDiff = std::min(xDiff, yDiff);
            int maxDiff = std::max(xDiff, yDiff);
            return (DIAGONAL_COST)*minDiff + (INLINE_COST)*(maxDiff-minDiff);
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

    if(VISUALIZE)
        Visualizer::waitForUser();

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
