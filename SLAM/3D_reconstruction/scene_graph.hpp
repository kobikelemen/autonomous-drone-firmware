#include <map>
// #include </Users/kobikelemen/Documents/programming/c++/opencv2/include/opencv2/opencv.hpp>
// #include "/usr/local/opt/opencv@2/include/opencv2/opencv.hpp"

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/sfm/triangulation.hpp>

struct Edge
{
    int connected_frame; 
    std::vector<cv::DMatch> matches;
    Edge(int c, std::vector<cv::DMatch> m) {
        connected_frame = c;
        matches = m;
    }
    // fundamental matrix, rotation ?
};


class Image
{
public:
    int frame_no;
    cv::Mat frame;
    std::vector<Edge> connections;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;
    Image(int frame_no,  std::vector<cv::KeyPoint> keypoints, cv::Mat descriptor, cv::Mat frame);
    void add_connection(int c, std::vector<cv::DMatch> m);
    std::vector<cv::DMatch> get_matches(int u);
    std::vector<int> get_connection_indexes();

};


class Scene_graph
{
    std::vector<Image> frames;
public:
    void add_frame(Image f);
    int keyframe_count;
    std::vector<Image> get_frames(void);
    Image* get_frame(int frame_num);
    Scene_graph();
    void add_connection(int frame1, int frame2, std::vector<cv::DMatch> matches);
    void initialise(Image img1, Image img2); // fundamental matrix + triangulation
};