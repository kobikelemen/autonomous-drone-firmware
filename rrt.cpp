#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

float max(std::vector<float> x);
float min(std::vector<float> x);


class Shape
{
public:
    // each vertex is relative from drone
    std::vector<Vector> boundary;

    Shape(std::vector<Vector> bound){
        boundary = bound;
    }

    float calc_interior_angle(Vector p1, Vector p2, Vector p3){
        Vector p21 = p2;
        p21.minus_vec(p1);
        Vector p23 = p2;
        p23.minus_vec(p3);
        float d = p21.dot_product(p23);
        float angle = acos(d/(p21.magnitude() * p23.magnitude()));
        return angle; // returns angle in radians I believe
    }

    std::vector<Shape> triangulate(){
        std::vector<Shape> triangles;
        std::vector<float> interior_angles;
        for (int i=1; i < boundary.size(); i+= 3){
            interior_angles.push_back(calc_interior_angle(
                boundary[i-1],
                boundary[i],
                boundary[i+1]
            ));
        }
        // complete rest of triangulation algo... (link here):
        // https://arxiv.org/pdf/1212.6038.pdf#:~:text=The%20ear%20clipping%20triangulation%20algorithm,newly%20formed%20triangle%20is%20valid.

    }


};


class Vector
{
    public:
    float x;
    float y;
    float z;
    Vector(float _x, float _y, float _z){
        x = _x;
        y = _y;
        z = _z;
    }

    float magnitude(){
        return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    }
    Vector unit_vec(){
        Vector unit_vec(
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
    void add_vec(Vector vec){
        x += vec.x;
        y += vec.y;
        z += vec.z;
    }

    void minus_vec(Vector vec){
        x -= vec.x;
        y -= vec.y;
        z -= vec.z;
    }

    float dot_product(Vector vec){
        return x * vec.x + y * vec.y + vec.z + z;
    }

    bool in_goal(std::vector<Vector> goal){
        std::vector<float> x_, y_ ,z_;
        for (int i=0; i < goal.size(); i++){
            x_.push_back(goal[i].x);
            y_.push_back(goal[i].y);
            z_.push_back(goal[i].z);
        }
        Vector maxim(max(x_), max(y_), max(z_));
        Vector minim(min(x_), min(y_), min(z_));
        if (x <= maxim.x && y <= maxim.y && x >= minim.x && y >= minim.y){
            return true;
        }
        return false;
    }

};


class Node
{
public:
    std::vector<Node> connections;
    Vector pos;
    int depth;
    Node(std::vector<Node> connections_, Vector pos_, int _depth){
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
    int nearest_node_index(Vector p)
    {
        //Point p1(node_list[0].pos.x, node_list[0].pos.y, node_list[0].pos.z);

        //Node* nearest = new Node(node_list[0].connections, node_list[0].pos, node_list[0].depth);
        //Node * nearest_ptr;
        int nearest_index = 0;
        Vector diff_vec(node_list[0].pos.x - p.x, node_list[0].pos.y - p.y, node_list[0].pos.z - p.z);
        float nearest_dist = diff_vec.magnitude();
        int i = 0;
        for (int i=1; i < node_list.size(); i++){
            //Node test_node = node_list[i];
            Vector dis(node_list[i].pos.x - p.x, node_list[i].pos.y - p.y, node_list[i].pos.z - p.z);
            float distance = dis.magnitude();
            if (distance < nearest_dist){
                nearest_index = i;
                //*nearest = test_node;
                //nearest_ptr = &nearest;
                nearest_dist = distance;
            }
        }
        //return nearest_ptr;
        return nearest_index;
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


Vector get_random_(std::vector<Vector> boundary, std::vector<Vector> obstacles)
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
    Vector p_max(max(x_boundaries), max(y_boundaries), max(z_boundaries));
    Vector p_min(min(x_boundaries), min(y_boundaries), min(z_boundaries));

    while (!check){
        x_check = rand_num(p_max.x, p_min.x);
        y_check = rand_num(p_max.y, p_min.y);
        z_check = rand_num(p_max.z, p_min.z);
        if (x_check < p_max.x && x_check > p_min.x && y_check < p_max.y && y_check > p_min.y){ //} && z_check > p_max.z && z_check < p_min.z){
            // havent checked if in obstacle yet ... or FOR Z!
            Vector p(x_check, y_check, z_check);
            return p;
        }
        //std::this_thread::sleep_for(std::chrono::nanoseconds(1e9));
        using namespace std::this_thread; // sleep_for, sleep_until
        using namespace std::chrono; // nanoseconds, system_clock, seconds

        // sleep_for(nanoseconds(1000000000));
        // std::cout << "x_check: " << x_check << std::endl << "y_check: " << y_check << std::endl << "z_check: " << z_check << std::endl;

    }
    Vector p_(0,0,0);
    return p_;
}


Vector get_random(std::vector<Vector> bound, std::vector<Vector> obstacles)
{
    //assuming boundary is ordered so each points are closest they can be to eachother

    Shape boundary(bound);
    std::vector<Shape> triangles = boundary.triangulate();

    //
}



Vector chain(Vector nearest, Vector new_p, float max_step_size)
{
    float magn;
    Vector diff_vec(new_p.x - nearest.x, new_p.y - nearest.y, new_p.z - nearest.z);
    if (diff_vec.magnitude() > max_step_size){
        magn = max_step_size;
        Vector unit_vec = diff_vec.unit_vec();
        unit_vec.scalar_mult(magn);
        diff_vec = unit_vec;
    }
    nearest.add_vec(diff_vec);
    Vector new_point = nearest;
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
        std::cout << "[" << G.node_list[i].pos.x << "," << G.node_list[i].pos.y << "]" << "," << std::endl; //"," << G.node_list[i].pos.z << "]" << "," << std::endl;
    }
    std::cout << "]";
}

std::vector<Node> rrt(
    std::vector<Vector> goal, Node start_node, int lim, Graph G, 
    float step_size, std::vector<Vector> boundary, 
    std::vector<Vector> obstacles
    )
{
    std::cout << "rrt... " << std::endl;
    int counter = 0;
    G.add_node(start_node);
    while (counter < lim){
        Vector new_p = get_random(boundary, obstacles);
        int nearest_index = G.nearest_node_index(new_p);
        Vector new_point = chain(G.node_list[nearest_index].pos, new_p, step_size);
        int depth = G.node_list[nearest_index].depth + 1;
        std::vector<Node> new_point_connections;
        new_point_connections.push_back(G.node_list[nearest_index]);
        Node new_node(new_point_connections, new_point, depth);
        G.node_list[nearest_index].add_connection(new_node);
        G.add_node(new_node);
        
        if (new_point.in_goal(goal)){
            std::cout << " GOALLLLL " << std::endl;
            std::vector<Node> path = get_path(G, new_node);
            print_graph(G);
            return path;
        }
        counter++;
    }
    std::cout << "counter end: " << counter << std::endl;
    std::vector<Node> pt;
    return pt;

}


int main()
{
    //test_rrt();
    return 0;
}