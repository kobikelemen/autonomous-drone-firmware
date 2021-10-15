#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

float max(std::vector<float> x);
float min(std::vector<float> x);


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

    bool in_goal(std::vector<Point> goal){
        // checks if point is in goal
        std::vector<float> x_, y_ ,z_;
        for (int i=0; i < goal.size(); i++){
            x_.push_back(goal[i].x);
            y_.push_back(goal[i].y);
            z_.push_back(goal[i].z);
        }
        Point maxim(max(x_), max(y_), max(z_));
        Point minim(min(x_), min(y_), min(z_));
        if (x < maxim.x && y < maxim.y && z < maxim.z && x > minim.x && y > minim.y && z > minim.z){
            return true;
        }
        return false;
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
    void add_connection(Node n){
        connections.push_back(n);
    }

};

class Graph
{
public:
    std::vector<Node> node_list;
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


float rand_num(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


Point get_random(std::vector<Point> boundary, std::vector<Point> obstacles)
{
// get random point within boudnary thats not in an obstacle
    std::vector<float> x_boundaries, y_boundaries, z_boundaries;
    for (int i=0; i < boundary.size(); i++){
        x_boundaries.push_back(boundary[i].x);
        y_boundaries.push_back(boundary[i].y);
        z_boundaries.push_back(boundary[i].z);
    }
    bool check = false;
    float x_check, y_check, z_check;
    Point p_max(max(x_boundaries), max(y_boundaries), max(z_boundaries));
    Point p_min(min(x_boundaries), min(y_boundaries), min(z_boundaries));

    while (!check){
        x_check = rand_num(p_max.x, p_min.x);
        y_check = rand_num(p_max.y, p_min.y);
        z_check = rand_num(p_max.z, p_min.z);
        //std::cout << "HIII" << std::endl;
        if (x_check < p_max.x && x_check > p_min.x && y_check < p_max.y && y_check > p_min.y){ //} && z_check > p_max.z && z_check < p_min.z){
            // havent checked if in obstacle yet ... or FOR Z!
            Point p(x_check, y_check, z_check);
            return p;
        }
        //std::this_thread::sleep_for(std::chrono::nanoseconds(1e9));
        using namespace std::this_thread; // sleep_for, sleep_until
        using namespace std::chrono; // nanoseconds, system_clock, seconds

        // sleep_for(nanoseconds(1000000000));
        // std::cout << "x_check: " << x_check << std::endl << "y_check: " << y_check << std::endl << "z_check: " << z_check << std::endl;

    }
    Point p_(0,0,0);
    return p_;
}

Point chain(Point nearest, Point new_p, float max_step_size)
{
    float magn;
    Point diff_vec(new_p.x - nearest.x, new_p.y - nearest.y, new_p.z - nearest.z);
    if (diff_vec.magnitude() > max_step_size){
        magn = max_step_size;
        Point unit_vec = diff_vec.unit_vec();
        unit_vec.scalar_mult(magn);
        diff_vec = unit_vec;
    }
    nearest.add_vec(diff_vec);
    Point new_point = nearest;
    return new_point;
}

std::vector<Node> get_path(Graph G, Node end_node)
{   
    int depth = end_node.depth;
    Node node = end_node;
    std::vector<Node> reverse_path;
    while (depth != 1){
        for (int i=0; i<node.connections.size(); i++){
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


void print_graph(Graph G){
    std::cout << " FOUND! " << std::endl;
    std::cout << "[";
    for (int i=0; i < G.size(); i++){
        std::cout << "[" << G.node_list[i].pos.x << "," << G.node_list[i].pos.y << "," << G.node_list[i].pos.z << "]" << "," << std::endl;
    }
    std::cout << "]";
}

std::vector<Node> rrt(
    std::vector<Point> goal, Node start_node, int lim, Graph G, 
    float step_size, std::vector<Point> boundary, 
    std::vector<Point> obstacles
    )
{
    std::cout << "rrt... " << std::endl;
    int counter = 0;
    G.add_node(start_node);
    //std::vector<Point> visited_pos;
    while (counter < lim){
        Point new_p = get_random(boundary, obstacles);
        Node *nearest;
        nearest = G.nearest_node(new_p);
        Point new_point = chain(nearest->pos, new_p, step_size);
        int depth = nearest->depth + 1;
        std::vector<Node> new_point_connections;
        new_point_connections.push_back(*nearest);
        Node new_node(new_point_connections, new_point, depth);
        nearest->add_connection(new_node);
        G.add_node(new_node);
        // if (new_point.in_goal(goal)){
        //     std::vector<Node> path = get_path(G, new_node);
        //     print_graph(G);
        //     return path;
        // }
        // counter++;

        std::vector<Node> hi;
        return hi;
    }
    std::cout << "counter end: " << counter << std::endl;
    std::vector<Node> pt;
    return pt;

}


void test_random()
{
    std::vector<Point> boundary;
    Point p1(0,0,0), p2(0,1,0), p3(1,0,0), p4(1,1,0);
    boundary.push_back(p1);
    boundary.push_back(p2);
    boundary.push_back(p3);
    boundary.push_back(p4);
    std::vector<Point> obstacles;
    std::cout << "YOOOO";
    Point r = get_random(boundary, obstacles);
    std::cout << r.x << " " << r.y << " " << r.z << std::endl;
}

void test_rrt()
{
    Point p1(6,6,0), p2(5,5,0), p3(5,6,0), p4(6,5,0), start_pos(0,0,0), b1(0,0,0), b2(20,20,0), b3(0,20,0), b4(20,0,0);
    std::vector<Point> goal; 
    goal.push_back(p1);
    goal.push_back(p2);
    goal.push_back(p3);
    goal.push_back(p4);
    std::vector<Node> connections;
    Node start_node(connections, start_pos, 1);
    std::cout << "HO";
    Graph G;
    float step_size = 0.1;
    int lim = 1000;
    std::vector<Point> boundary;
    boundary.push_back(b1);
    boundary.push_back(b2);
    boundary.push_back(b3);
    boundary.push_back(b4);
    std::vector<Point> obstacles;
    rrt(goal, start_node, lim, G, step_size, boundary, obstacles);
}




int main()
{
    test_rrt();
    return 0;
}