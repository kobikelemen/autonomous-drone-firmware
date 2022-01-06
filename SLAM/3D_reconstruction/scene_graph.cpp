#include "scene_graph.hpp"
#include <vector>

void Scene_graph::initialise(Image img1, Image img2)
{
    std::vector<cv::DMatch> matches = img1.get_matches(img2.frame_no);
    std::vector<cv::Point2f> points1, points2;

    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it) {
        float x = img1.keypoints[it->queryIdx].pt.x;
        float y = img1.keypoints[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x, y));
        x = img2.keypoints[it->trainIdx].pt.x;
        y = img2.keypoitns[it->trainIdx].pt.y;
        img2.keypoints.push_back(cv::Point2f(x, y));
    }

    cv::Mat fundamental = cv::findFundamentalMat(points1, points2, method=cv::RANSAC, 0.9, 1);
    
}