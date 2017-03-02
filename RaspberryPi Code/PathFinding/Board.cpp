class Board{
    public:
        Board(Vector2D* obstacles, int obstaclesNumber, Vector2D _bsize) : bsize(_bsize){
            initializeBoard();
            initializeObstacles(obstacles, obstaclesNumber);
            initializeNeighbours();
        };

        Node getNode(Vector2D coordinates){
            return board[coordinates.x][coordinates.y];
        }

        Node* getNodePtr(Vector2D coordinates){
            return &board[coordinates.x][coordinates.y];
        }

        int getWidth(){
            return bsize.x;
        }

        int getHeight(){
            return bsize.y;
        }

        bool isInBounds(Vector2D p){
            return (p.x >= 0 && p.x < bsize.x && p.y >= 0 && p.y < bsize.y);
        }

        void reset(){
            for(int i=0; i<bsize.x; i++){
                for(int j=0; j<bsize.y; j++){
                    board[i][j].parent = NULL;
                    board[i][j].costSoFar = INT_MAX;
                    board[i][j].heuristic = INT_MAX;
                    board[i][j].wasClosed = false;
                }
            }
        }

    private:
        Vector2D bsize;
        Node** board;

        void initializeBoard(){
            board = new Node*[bsize.x];
                for(int i=0; i<bsize.x; i++){
                    board[i] = new Node[bsize.y];

                    for(int j=0; j<bsize.y; j++)
                        board[i][j] = Node(Vector2D(i, j));
                }
        }

        void initializeObstacles(Vector2D* obstacles, int obstaclesNumber){
             for(int i=0; i<obstaclesNumber; i++)
                board[obstacles[i].x][obstacles[i].y].isObstacle = true;
        }

        void initializeNeighbours(){
            for(int i=0; i<bsize.x; i++){
                for(int j=0; j<bsize.y; j++){
                    if(j > 0)
                        board[i][j].inlineNeighbours.push_back(&board[i][j-1]);
                    if(i+1 < bsize.x)
                        board[i][j].inlineNeighbours.push_back(&board[i+1][j]);
                    if(j+1 < bsize.y)
                        board[i][j].inlineNeighbours.push_back(&board[i][j+1]);
                    if(i > 0)
                        board[i][j].inlineNeighbours.push_back(&board[i-1][j]);

                    if(i > 0 && j > 0)
                        board[i][j].diagonalNeighbours.push_back(&board[i-1][j-1]);
                    if(i+1 < bsize.x && j > 0)
                        board[i][j].diagonalNeighbours.push_back(&board[i+1][j-1]);
                    if(i+1 < bsize.x && j+1 < bsize.y)
                        board[i][j].diagonalNeighbours.push_back(&board[i+1][j+1]);
                    if(i > 0 && j+1 < bsize.y)
                        board[i][j].diagonalNeighbours.push_back(&board[i-1][j+1]);
                }
            }
        }
};
