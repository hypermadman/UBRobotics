#include <list>
#include <climits>
#include <cstdio>

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

            openNodes.clear();

            Node* startNode = board.getNodePtr(start);
            startNode->costSoFar = 0;
            startNode->heuristic = getHeuristic(startNode->position, goal);
            openNodes.push_back(startNode);

            do{
                if(VISUALIZE)
                   Visualizer::visualize(board, goal);

                currNode = openNodes.front();
                openNodes.pop_front();
                currNode->wasClosed = true;
                popClosedNodes(openNodes);
                updateNodes(currNode->inlineNeighbours, INLINE_COST, goal);
                updateNodes(currNode->diagonalNeighbours, DIAGONAL_COST, goal);

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
        std::list<Node*> openNodes;
        Node* currNode;

        void popClosedNodes(std::list<Node*> &nodes){
            while(!nodes.empty() && nodes.front()->wasClosed)
                nodes.pop_front();
        }

        void updateNodes(std::list<Node*> &nodes, int cost, Vector2D &goal){
            for(std::list<Node*>::iterator it = nodes.begin(); it != nodes.end(); it++){
                if((*it)->isObstacle || (*it)->wasClosed)
                    continue;

                (*it)->heuristic = getHeuristic((*it)->position, goal);
                int potentialCostSoFar = currNode->costSoFar + INLINE_COST;

                if(potentialCostSoFar < (*it)->costSoFar){
                    (*it)->costSoFar = potentialCostSoFar;
                    (*it)->parent = currNode;
                    insertIntoOpenNodes(*it);
                }
            }
        }

        int getHeuristic(Vector2D &start, Vector2D &goal){
            int xDiff = std::abs(start.x - goal.x);
            int yDiff = std::abs(start.y - goal.y);
            int minDiff = std::min(xDiff, yDiff);
            int maxDiff = std::max(xDiff, yDiff);
            return (DIAGONAL_COST)*minDiff + (INLINE_COST)*(maxDiff-minDiff);
        }

        void insertIntoOpenNodes(Node* node){
            for(std::list<Node*>::iterator openIt = openNodes.begin();; openIt++){
                if(openIt == openNodes.end()){
                    openNodes.push_back(node);
                    break;
                }

                if(node->costSoFar+node->heuristic <= (*openIt)->costSoFar+(*openIt)->heuristic){
                    openNodes.insert(openIt, node);
                    break;
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
