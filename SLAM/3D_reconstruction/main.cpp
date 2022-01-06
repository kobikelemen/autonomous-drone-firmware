#include "scene_graph.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include <iostream>


// need to link opencv:
// THIS ONE: g++ -std=c++11 $(pkg-config --cflags --libs opencv) main.cpp -o main
// then run executable with ./main
// (pkg-config --cflags --libs opencv has links)


void save_frames(Scene_graph &g)
{
    cv::VideoCapture house_tour("/Users/kobikelemen/Documents/programming/c++/open_test/house_tour.mp4");
    cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
    cv::Ptr<cv::ORB> detector = cv::ORB::create(2000);
    std::vector<Image> first_two;

    for (int frame_num=0; frame_num <= 30; frame_num++) {
        if (frame_num % 30 == 0) {
            cv::Mat f;
            house_tour >> f;
            std::vector<cv::KeyPoint> kp;
            cv::Mat desc;
            detector->detectAndCompute(f, cv::noArray(), kp, desc);
            int t = frame_num/30;
            Image img(t, kp, desc, f);
            first_two.push_back(img);
            g.add_frame(img);
        }
    }
    g.initialise(first_two[0], first_two[1]);

    for (int frame_num=31; frame_num < house_tour.get(cv::CAP_PROP_FRAME_COUNT); frame_num++) {
        if (frame_num % 30 == 0) {
            cv::Mat f;
            house_tour >> f;
            std::vector<cv::KeyPoint> kp;
            cv::Mat desc;
            detector->detectAndCompute(f, cv::noArray(), kp, desc);
            int t = frame_num/30;
            Image img(t, kp, desc, f);
            g.add_frame(img);

            cv::Mat imgkp;
            cv::drawKeypoints(f, kp, imgkp);
            cv::imshow("img", imgkp);

            if (cv::waitKey(30) == 27) { // break using ESC key
                break;
            }
        }
    }
    cv::destroyAllWindows();
}

bool check(cv::DMatch i, cv::DMatch j) {
    return i.distance < j.distance;
}

void image_correspondances(Scene_graph &g)
{
    std::vector<Image> imgs = g.get_frames();
    cv::BFMatcher matcher(cv::NORM_HAMMING2, true);

    for (Image img : imgs) {
        for (Image check_img : imgs) {
            if (img.frame_no != check_img.frame_no) {
                std::vector<cv::DMatch> matches;
                matcher.match(img.descriptor, check_img.descriptor, matches);
                std::sort(matches.begin(),matches.end(), check);
                if (matches.front().distance < 20) {
                    g.add_connection(img.frame_no, check_img.frame_no);
                }
                // MISSING GEOMETRIC VERIFICATINO USING RANSAC
                // MAYBE DIFFERENT MATCHER HANDLES THAT?
            }
        }
    }
}






int main()
{
    

    // cv::Mat img = cv::imread("../open_test/apple.jpeg");
    // std::vector<cv::Point2f> p1, p2;
    // cv::Mat inliers;
    // cv::Mat essential = cv::findEssentialMat(p1, p2);
    Scene_graph g;
    save_frames(g);
    return 0;

}




