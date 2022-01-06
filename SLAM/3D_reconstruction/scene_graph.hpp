#include <map>
// #include </Users/kobikelemen/Documents/programming/c++/opencv2/include/opencv2/opencv.hpp>
// #include "/usr/local/opt/opencv@2/include/opencv2/opencv.hpp"

#include <vector>



class Image
{
public:
    int frame_no;
    cv::Mat frame;
    std::vector<int> connections;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;
    Image(int frame_no,  std::vector<cv::KeyPoint> keypoints, cv::Mat descriptor, cv::Mat frame);
    void add_connection(int c);
    std::vector<int> get_connection_indexes();

};


class Scene_graph
{
    std::vector<Image> frames;
public:
    void add_frame(Image f);
    std::vector<Image> get_frames(void);
    Image* get_frame(int frame_num);
    Scene_graph();
    void add_connection(int f1, int f2);
    void initialise(Image img1, Image img2); // fundamental matrix + triangulation
};