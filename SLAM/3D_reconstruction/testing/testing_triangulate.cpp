#include "../scene_graph.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp> 
#include <string>
#include <iostream>




int main()
{
    std::vector<Image> test_imgs;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(1000);
    // Scene_graph G;
    for (int i=24; i < 496; i+=8){
        cv::Mat f = cv::imread("../frames/frame" + std::to_string(i) + ".jpg");
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc;
        detector->detectAndCompute(f, cv::noArray(), kp, desc);
        Image img(i, kp, desc, f);
        test_imgs.push_back(img);
        cv::Mat imgkp;
        cv::drawKeypoints(f, kp, imgkp);
        cv::imwrite("/Users/kobikelemen/Documents/programming/c++/open_test/testing/test_frames/framekp" + std::to_string(i)+".jpg", imgkp);
        // G.initialise(test_imgs[i-8], test_imgs[8], i);
    }
    std::cout << " YO " << std::endl;

    for (int i=24; i < 496; i+=8) {
        std::cout << i << std::endl;
        Scene_graph G;
        G.initialise(test_imgs[i-8], test_imgs[8], i);
    }


}