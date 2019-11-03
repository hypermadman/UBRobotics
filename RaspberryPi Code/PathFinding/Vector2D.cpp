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
