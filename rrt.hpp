#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>

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
public:
    std::vector<Node> connections;
    Point pos;
    int depth;
    Node(std::vector<Node> connections_, Point pos_, int _depth){
        connections = connections_;
        pos = pos_;
        depth = _depth;
    }
    void add_connections(Node n){
        connections.push_back(n);
    }

};

class Graph
{
    std::vector<Node> node_list;
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
    int size(){
        return node_list.size();
    }

};


float max(std::vector<float> x){
    float max = 0;
    for (int i=0; i < x.size(); i++){
        if (x[i] > max){
            max = x[i];
        }
    }
    return max;
}

float min(std::vector<float> x){
    float min = 1e5;
    for (int i=0; i < x.size(); i++){
        if (x[i] < min){
            min = x[i];
        }
    }
    return min;
}


Point get_random(std::vector<Point> boundary, std::vector<Point> obstacles)
{
// get random point within boudnary thats not in an obstacle
    std::vector<float> x_boundaries;
    std::vector<float> y_boundaries;
    std::vector<float> z_boundaries;
    for (int i=0; i < boundary.size(); i++){
        x_boundaries.push_back(boundary[i].x);
        y_boundaries.push_back(boundary[i].y);
        z_boundaries.push_back(boundary[i].z);
    }
    bool check = false;
    float x_check, y_check, z_check;
    Point p_max(max(x_boundaries), max(y_boundaries), max(z_boundaries));
    Point p_min(min(x_boundaries), max(y_boundaries), max(z_boundaries));

    while (!check){
        x_check = rand() % (p_max.x - p_min.x);
        x_check += p_min.x;
        y_check = rand() % (p_max.y - p_min.y);
        y_check += p_min.y;
        z_check = rand() % (p_max.z - p_min.z);
        z_check += p_min.z;

        if (x_check < p_max.x && x_check > p_min.x && y_check < p_max.y && y_check > p_min.y && z_check > p_max.z && z_check < p_min.z){
            // havent checked if in obstacle yet ... !
            Point p(x_check, y_check, z_check);
            return p;
        }


    }
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
    Point new_point = nearest.add_vec(diff_vec);
    return new_point;
}

std::vector<Node> get_path(Graph G, Node end_node)
{   
    int depth = end_node.depth;
    Node node = end_node;
    std::vector<Node> reverse_path;
    while (depth != 1){
        for (int i=0; i<node.connections.size(), i++){
            Node check_node = node.connections[i];
            if (check_node.depth == node.depth - 1) {
                node = check_node;
                depth = node.depth;
                reverse_path.push_back(node);
            }
        }
    }
    std::reverse(reverse_path.begin(), reverse_path.end());
    return reverse_path; // not reverse anymore ...
}




std::vector<Node> rrt(
    Node goal, Node start_node, int lim, Graph G, 
    float step_size(std::vector<Point> boundary, 
    std::vector<Point> obstacles
    )
{
    int counter = 0;
    start_node.depth = 1;
    G.add_node(start_node);
    while (counter < lim){
        Point new_p = get_random(boundary, obstacles);
        Node *nearest;
        nearest = G.nearest_node(new_p);
        Point new_point = chain(*nearest.pos, new_p, step_size);
        std::vector<Node> new_point_connections = {*nearest};
        Node new_node(new_point_connections, new_point);
        *nearest.add_connection(new_node);
        new_node.depth = *nearest.depth + 1;
        G.add_node(new_node);
        if (new_point.in_goal()){
            std::vector<Node> path = get_path(G, new_node);
            return path;
        }
        counter++;
    }

}