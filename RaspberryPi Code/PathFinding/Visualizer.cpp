#include <unistd.h>
#include <cstdlib>

#define DELAY 10000

class Visualizer{
    public:
        static void waitForUser(){
            usleep(DELAY*500);
        }

        static void visualize(Node** board, Vector2D boardSize, Vector2D goal){
            std::system("clear");
            printBoard(board, boardSize, goal);
            usleep(DELAY);
        }

    private:
        static void printBoard(Node** board, Vector2D boardSize, Vector2D goal){
            for(int i=0; i<boardSize.y; i++){
                for(int j=0; j<boardSize.x; j++){
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
};
