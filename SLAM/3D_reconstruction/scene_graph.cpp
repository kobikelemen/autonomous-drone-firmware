
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
        y = img2.keypoints[it->trainIdx].pt.y;
        img2.keypoints.push_back(cv::Point2f(x, y));
    }

    cv::Mat fundamental = cv::findFundamentalMat(points1, points2, method=cv::RANSAC, 0.9, 1);

}


void Scene_graph::add_connection(int frame1, int frame2, std::vector<cv::DMatch> matches)
{
    Image *im1 = get_frame(frame1);
    Image *im2 = get_frame(frame2);
    // wwould be faster to get both frames in one go
    // makes more sense to pass pointer to img instead of searching for it here...
    im1->add_connection(frame1, matches);
    im2->add_connection(frame2, matches);

}


Image* Scene_graph::get_frame(int frame_num)
{
    Image *f = &frames[frame_num/keyframe_count];
    return f;
}


void Scene_graph::add_frame(Image f)
{
    frames.push_back(f);
}


std::vector<Image> Scene_graph::get_frames(void)
{
    return frames;
}






void Image::add_connection(int c, std::vector<cv::DMatch> m)
{
    Edge edge(c, m);
    connections.push_back(edge);
    // could maybe improve efficiency by adding edges in order 
    // so when searching for edge can use bianry search
}