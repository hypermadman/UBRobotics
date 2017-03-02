#include <unistd.h>
#include <cstdlib>

#define DELAY 10000

class Visualizer{
    public:
        static void waitForUser(){
            usleep(DELAY*200);
        }

        static void visualize(Board board, Vector2D goal){
            std::system("clear");
            printBoard(board, goal);
            usleep(DELAY);
        }

    private:
        static void printBoard(Board board, Vector2D goal){
            for(int i=0; i<board.getHeight(); i++){
                for(int j=0; j<board.getWidth(); j++){
                    Vector2D coordinates = Vector2D(j, i);
                    if(coordinates == goal)
                        printf("!");
                    else if(board.getNode(coordinates).wasClosed)
                        printf("x");
                    else if(board.getNode(coordinates).isObstacle)
                        printf("0");
                    else
                        printf(".");
                }

                printf("\n");
            }
        }
};
