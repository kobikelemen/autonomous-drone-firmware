#include <map>
// #include <opencv2/opencv.hpp>
#include <vector>



class Image
{
    int frame_no;
    std::vector<int> connections;
    // cv::keypoint
    Image();
    void add_connection(int c);
    std::vector<int> get_connection_indexes();

};


class Scene_graph
{
    Scene_graph();
};