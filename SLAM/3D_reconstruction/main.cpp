#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include "scene_graph.hpp"
#include <iostream>
#include <string>


#include <chrono>
#include <thread>

// need to link opencv:
// THIS ONE: g++ -std=c++11 $(pkg-config --cflags --libs opencv) main.cpp -o main

// I THINK THIS WORKS:: g++ -std=c++11 $(pkg-config --cflags --libs opencv4) main.cpp -o main
// (NOT PREVIOUS)


void image_correspondances(Scene_graph &g, Image img);

void save_frames(Scene_graph &g)
{
    cv::VideoCapture house_tour("/Users/kobikelemen/Documents/programming/c++/open_test/trimmed_vid.mp4");
    cv::namedWindow("img", cv::WINDOW_AUTOSIZE);
    cv::Ptr<cv::ORB> detector = cv::ORB::create(1000);
    std::vector<Image> first_two;
    g.keyframe_count = 8;
    for (int frame_num=0; frame_num <= g.keyframe_count; frame_num++) {
        if (frame_num % g.keyframe_count == 0) {
            cv::Mat f;
            house_tour >> f;
            std::vector<cv::KeyPoint> kp;
            cv::Mat desc;
            detector->detectAndCompute(f, cv::noArray(), kp, desc);
            int t = frame_num/30;
            Image img(t, kp, desc, f);
            first_two.push_back(img);
            g.add_frame(img);
            image_correspondances(g, img);
        }
    }
    // g.initialise(first_two[0], first_two[1]);
    // std::cout << "YOOOOOO";
    // using namespace std::this_thread; // sleep_for, sleep_until
    // using namespace std::chrono; // nanoseconds, system_clock, seconds

    // sleep_for(nanoseconds(1000000000));


    for (int frame_num=g.keyframe_count+1; frame_num < house_tour.get(cv::CAP_PROP_FRAME_COUNT); frame_num++) {
        if (frame_num % g.keyframe_count == 0) {
            cv::Mat f;
            house_tour >> f;
            std::vector<cv::KeyPoint> kp;
            cv::Mat desc;
            detector->detectAndCompute(f, cv::noArray(), kp, desc);
            int t = frame_num/g.keyframe_count;
            Image img(t, kp, desc, f);
            g.add_frame(img);
            image_correspondances(g, img);

            cv::Mat imgkp;
            cv::drawKeypoints(f, kp, imgkp);
            cv::imshow("img", imgkp);
            // cv::imwrite("/Users/kobikelemen/Documents/programming/c++/open_test/frames/frame" + std::to_string(frame_num)+".jpg", imgkp);
            cv::imwrite("/Users/kobikelemen/Documents/programming/c++/open_test/frames/frame" + std::to_string(frame_num)+".jpg", f);
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

//std::vector<cv::DMatch> 
void image_correspondances(Scene_graph &g, Image img)
{
    std::vector<Image> imgs = g.get_frames();
    cv::BFMatcher matcher(cv::NORM_HAMMING2, true);
    // makes more sense to pass pointer to img to add_connection instead of searching for it again
    for (Image check_img : imgs) {
        if (img.frame_no != check_img.frame_no) {
            std::vector<cv::DMatch> matches;
            matcher.match(img.descriptor, check_img.descriptor, matches);
            std::sort(matches.begin(),matches.end(), check);
            if (matches.front().distance < 20) {
                g.add_connection(img.frame_no, check_img.frame_no, matches);
            } 
                // MISSING GEOMETRIC VERIFICATINO USING RANSAC
                // MAYBE DIFFERENT MATCHER HANDLES THAT?
        }
    }
    // return matches;

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




