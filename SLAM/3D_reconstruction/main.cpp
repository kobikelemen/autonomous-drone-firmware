#include "scene_graph.hpp"
// #include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include <iostream>


// need to link opencv:
// THIS ONE: g++ -std=c++11 $(pkg-config --cflags --libs opencv) main.cpp -o main
// then run executable with ./main
// (pkg-config --cflags --libs opencv has links)


void save_frames()
{
    cv::VideoCapture house_tour("/Users/kobikelemen/Documents/programming/c++/open_test/house_tour.mp4");
    cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
    for (int frame_num=0; frame_num < house_tour.get(cv::CAP_PROP_FRAME_COUNT); frame_num++) {
        cv::Mat f;
        house_tour >> f;
        cv::imshow("img", f);
        if (cv::waitKey(30) == 27) { // break using ESC key
            break;
        }
    }
    cv::destroyAllWindows();
}




int main()
{
    

    // cv::Mat img = cv::imread("../open_test/apple.jpeg");
    // std::vector<cv::Point2f> p1, p2;
    // cv::Mat inliers;
    // cv::Mat essential = cv::findEssentialMat(p1, p2);
    save_frames();
    return 0;

}
