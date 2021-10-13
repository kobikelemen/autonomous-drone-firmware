#include <math.h>
#include <vector>
#include <iostream>


class Point
{
    public:
    float x;
    float y;
    float z;
    Point(float _x, float _y, float _z){
        x = _x;
        y = _y;
        z = _z;
    }
    Point(){
    }

    float magnitude(){
        return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    }
    Point unit_vec(){
        Point unit_vec(
        x/sqrt(pow(x,2) + pow(y,2) + pow(z,2)),
        y/sqrt(pow(x,2) + pow(y,2) + pow(z,2)),
        z/sqrt(pow(x,2) + pow(y,2) + pow(z,2))
        );
        return unit_vec;
    }
    void scalar_mult(float scalar){
        x = scalar * x;
        y = scalar * y;
        z = scalar * z;
    }
    void add_vec(Point vec){
        x += vec.x;
        y += vec.y;
        z += vec.z;
    }

    bool in_goal(){
        // checks if point is in goal
    }

};


class Node
{
    vector<Node> connections;
public:
    Position pos;
    int depth;
    Node(vector<Node> connections_, Position pos_, int _depth){
        connections = connections_;
        pos = pos_;
        depth = _depth;
    }
    void add_connections(Node n){
        connections.pushback(n);
    }

};

class Graph
{
    vector<Node> node_list;
public:
    Node *nearest_node(Point p)
    {
        Node nearest = node_list[0];
        Node * nearest_ptr;
        Point diff_vec(nearest.pos.x - p.x, nearest.pos.y - p.y, nearest.pos.z - p.z);
        float nearest_dist = diff_vec.magnitude();
        for (int i=1; i < node_list.size(); i++){
            Node test_node = node_list[i];
            Point dis(test_node.pos.x - p.x, test_node.pos.y - p.y, test_node.pos.z - p.z);
            float distance = dis.magnitude();
            if (distance < nearest_dist){
                nearest = test_node;
                nearest_ptr = &nearest;
                nearest_dist = distance;
            }
        }
        return nearest_ptr;
    }

    void add_node(Node n){
        node_list.push_back(n);
        // add connections to other nodes aswell
        for (int i=0; i < node_list.size(); i++){
            
        }
    }

};


Point get_random(vector<Point> boundary, vector<Point> obstacles)
{
// get random point within boudnary thats not in an obstacle
}

Point chain(Point nearest, Point new_p, float max_step_size)
{
    float magn;
    Point diff_vec(new_p.x - nearest.x, new_p.y - nearest.y, new_p.z - nearest.z);
    if (diff_vec.magnitude() > max_step_size){
        magn = max_step_size;
        Point unit_vec = diff_vec.unit_vec();
        diff_vec = unit_vec.scalar_mult(magn);
    }
    Point new_point = nearest_node.add_vec(diff_vec);
    return new_point;
}

vector<Node> get_path(Graph G, Node end_node)
{

}




Graph rrt(Node goal, int lim, Graph G, float step_size, Node start_node)
{
    int counter = 0;
    start_node.depth = 1;
    G.add_node(start_node);
    while (counter < lim){
        Point new_p = get_random(boundary, obstacles);
        Node *nearest;
        nearest = G.nearest_node(new_p);
        Point new_point = chain(*nearest.pos, new_p, step_size);
        vector<Node> new_point_connections = {*nearest};
        Node new_node(new_point_connections, new_point);
        *nearest.add_connection(new_node);
        new_node.depth = *nearest.depth + 1;
        G.add_node(new_node);
        if (new_point.in_goal()){
            return G;
        }
        counter++;
    }
}