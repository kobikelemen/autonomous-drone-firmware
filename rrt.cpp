#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

float max(std::vector<float> x);
float min(std::vector<float> x);



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
    Vector(){}

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



class Shape
{
public:
    // each vertex is relative from drone
    std::vector<Vector> bound_list;

    Shape(std::vector<Vector> bound){
        bound_list = bound;
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

    // check if point is inside triangle
    float sign(Vector p1, Vector p2, Vector p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    bool check_in_triangle(Vector pt, Vector v1, Vector v2, Vector v3)
    {
        float d1, d2, d3;
        bool has_neg, has_pos;
        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);
        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(has_neg && has_pos);
    }

    //

    void remove(Vector p){
        bool test = false;
        int i=0;
        while (!test){
            if (bound_list[i].x == p.x && bound_list[i].y == p.y && bound_list[i].z == p.z){
                std::vector<Vector> b;
                for (int j=0; j < bound_list.size(); j++){
                    if (i != j){
                        std::cout << "i" << std::endl;
                        Vector p(bound_list[j].x, bound_list[j].y, bound_list[j].z);
                        b.push_back(p);
                    }
                }
                bound_list = b;
                test = true;
            }
            i++;
        }
    }


    std::vector<Shape> triangulate(){
        Shape bound_copy(bound_list);
        std::vector<Shape> triangles;
        std::vector<float> interior_angles;
        for (int i=1; i < bound_copy.bound_list.size()-1; i++){
            Vector p1 = bound_copy.bound_list[i-1];
            Vector p2 = bound_copy.bound_list[i];
            Vector p3 = bound_copy.bound_list[i+1];
            float angle = calc_interior_angle(p1, p2, p3);
            std::cout << "angle: " << angle << std::endl;
            if (angle < M_PI/2){
                bool check = false;
                for (int j=0; j < bound_copy.bound_list.size(); j++){
                    if (j!=i-1 && j!=i && j!=i+1){
                        check = check_in_triangle(bound_copy.bound_list[j],p1, p2,p3);
                        break;
                    }
                }
                if (!check){
                    std::vector<Vector> tri;
                    tri.push_back(p1);
                    tri.push_back(p2);
                    tri.push_back(p3);
                    Shape new_triangle(tri);
                    triangles.push_back(new_triangle);
                } 
            }
        }
    return triangles;
        // complete rest of triangulation algo... (link here):
        // https://arxiv.org/pdf/1212.6038.pdf#:~:text=The%20ear%20clipping%20triangulation%20algorithm,newly%20formed%20triangle%20is%20valid.

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

    //  UNFINISHED...
    Vector rnd;
    return rnd;
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

void test_triangulate(){
    std::vector<Vector> v;
    Vector p1(0,0,0), p2(1,2,0), p3(1,0,0), p4(-2,1,0);
    v.push_back(p1);
    v.push_back(p2);
    v.push_back(p3);
    v.push_back(p4);
    Shape s(v);
    std::vector<Shape> tris = s.triangulate();
    std::cout << "tris size: " << tris.size();
    for (int i=0; i < tris.size(); i++){
        std::cout << "triangle " << i << std::endl;
        for (int j=0; j < tris[i].bound_list.size(); j++){
            std::cout << "(" << tris[i].bound_list[j].x << "," << tris[i].bound_list[j].y << "," << tris[i].bound_list[j].z << ")";
        }
    }

}

void test_calc_angle(){
    std::vector<Vector> v;
    Vector p1(0,0,0), p2(1,0,0), p3(1,1,0), p4(0,1,0);
    v.push_back(p1);
    Shape s(v);
    float angle = s.calc_interior_angle(p1,p2,p3);
    std::cout << angle;
}




int main()
{
    //test_rrt();
    test_triangulate();
    // test_calc_angle();
    return 0;
}