#include <iostream>
#include <vector>



void test_random()
{
    std::vector<Vector> boundary;
    Vector p1(0,0,0), p2(0,1,0), p3(1,0,0), p4(1,1,0);
    boundary.push_back(p1);
    boundary.push_back(p2);
    boundary.push_back(p3);
    boundary.push_back(p4);
    std::vector<Vector> obstacles;
    std::cout << "YOOOO";
    Vector r = get_random(boundary, obstacles);
    std::cout << r.x << " " << r.y << " " << r.z << std::endl; // good
}

void test_rrt()
{
    Vector p1(8,8,0), p2(9,8,0), p3(9,9,0), p4(8,9,0), start_pos(0,0,0), b1(0,0,0), b2(0,10,0), b3(10,0,0), b4(10,10,0);
    std::vector<Vector> goal; 
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
    std::vector<Vector> boundary;
    boundary.push_back(b1);
    boundary.push_back(b2);
    boundary.push_back(b3);
    boundary.push_back(b4);
    std::vector<Vector> obstacles;
    rrt(goal, start_node, lim, G, step_size, boundary, obstacles);
} // good but not tested thoroughly ....

void test_nearest()
{
    Graph G;
    Vector p(0.1,0.7,0), p1(0,0,0), p2(0,1,0), p3(1,1,0), p4(1,0,0);
    std::vector<Node> connections;
    int depth = 1;
    Node n1(connections, p1, depth);
    Node n2(connections, p2, depth);
    Node n3(connections, p3, depth);
    Node n4(connections, p4, depth);
    G.add_node(n1);
    G.add_node(n2);
    G.add_node(n3);
    G.add_node(n4);
    int nearest_index = G.nearest_node_index(p);
    std::cout << "NEAREST: " << nearest_index << std::endl;
    std::cout << "nearest pos: " << G.node_list[nearest_index].pos.x << " " << G.node_list[nearest_index].pos.y << " " << G.node_list[nearest_index].pos.z << std::endl;
}           // good

void test_chain()
{

    Vector nearest(1,1,0);
    Vector new_p(1,0,0);
    float max_step_size = 2;
    Vector cha = chain(nearest, new_p, max_step_size);
    std::cout << "OUTPUT: " << cha.x << " " << cha.y << " " << cha.z << std::endl;
}       // good

void test_in_goal()
{
    Vector p_test(0.5,0.5,0), g1(0,0,0), g2(0,1,0), g3(1,0,0), g4(1,1,0);
    std::vector<Vector> goal;
    goal.push_back(g1);
    goal.push_back(g2);
    goal.push_back(g3);
    goal.push_back(g4);
    bool output = p_test.in_goal(goal);
    std::cout << "in goal: " << output << std::endl;
}       // good