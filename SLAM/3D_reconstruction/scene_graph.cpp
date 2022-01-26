
#include "scene_graph.hpp"


// WHEN CREATING THE 3D MESH, REFER TO PG 89 OF ALGO DESIGN MANUAL 
// -> CAN USE PRIORITY QUEUES TO OPTIMISE (BST)



cv::Vec3d triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2) {
// system of equations assuming image=[u,v] and X=[x,y,z,1]
  // from u(p3.X)= p1.X and v(p3.X)=p2.X
    cv::Matx43d A(u1(0)*p1.at<double>(2, 0) - p1.at<double>(0, 0),
    u1(0)*p1.at<double>(2, 1) - p1.at<double>(0, 1),
    u1(0)*p1.at<double>(2, 2) - p1.at<double>(0, 2),
    u1(1)*p1.at<double>(2, 0) - p1.at<double>(1, 0),
    u1(1)*p1.at<double>(2, 1) - p1.at<double>(1, 1),
    u1(1)*p1.at<double>(2, 2) - p1.at<double>(1, 2),
    u2(0)*p2.at<double>(2, 0) - p2.at<double>(0, 0),
    u2(0)*p2.at<double>(2, 1) - p2.at<double>(0, 1),
    u2(0)*p2.at<double>(2, 2) - p2.at<double>(0, 2),
    u2(1)*p2.at<double>(2, 0) - p2.at<double>(1, 0),
    u2(1)*p2.at<double>(2, 1) - p2.at<double>(1, 1),
    u2(1)*p2.at<double>(2, 2) - p2.at<double>(1, 2));
    cv::Matx41d B(
        p1.at<double>(0, 3) - u1(0)*p1.at<double>(2,3),
        p1.at<double>(1, 3) - u1(1)*p1.at<double>(2,3),
        p2.at<double>(0, 3) - u2(0)*p2.at<double>(2,3),
        p2.at<double>(1, 3) - u2(1)*p2.at<double>(2,3)
        );
  cv::Vec3d X;
  cv::solve(A, B, X, cv::DECOMP_SVD);
  return X;
}


void Scene_graph::initialise(Image img1, Image img2, int num)
{
    std::vector<cv::DMatch> matches = img1.get_matches(img2);
    std::vector<cv::Point2f> points1, points2;
    
    // temporary -->
    // cv::Mat matchImage;
    // cv::namedWindow("img1");
    // cv::drawMatches(img1.frame, img1.keypoints, img2.frame, img2.keypoints, matches, matchImage, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // cv::imwrite("matches.jpg", matchImage);
    // <--

    // convert keypoints to cv::point2f
    for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it) {
        float x = img1.keypoints[it->queryIdx].pt.x;
        float y = img1.keypoints[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x, y));
        x = img2.keypoints[it->trainIdx].pt.x;
        y = img2.keypoints[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x, y));
    }

    cv::Mat fundamental = cv::findFundamentalMat(points1, points2, cv::RANSAC, 0.9, 1);
    cv::Mat projection1, projection2; // camera matrices
    cv::sfm::projectionsFromFundamental(fundamental, projection1, projection2); // takes first frame as origin
    
    std::vector<cv::Vec3d> points3D;
    std::vector<cv::Mat> projections;
    std::vector<std::vector<cv::Point2f>> points2D;
    points2D.push_back(points1);
    points2D.push_back(points2);
    projections.push_back(projection1);
    projections.push_back(projection2);
    // std::cout << "size points1 and points2: "<< points1.size() << " " << points2.size() << std::endl;
    // std::cout << "size points2D: "<< points2D.size() << std::endl;
    // std::cout << "for each points2D: "<< std::endl;
    // for (int i=0; i < points2D.size(); i++) {
    //     std::cout << points2D[i].size() << std::endl;
    // }

    for (int i=0; i < points1.size(); i++) {
        cv::Vec2d one;
        cv::Vec2d two;
        one(0) = points1[i].x;
        one(1) = points1[i].y;
        two(0) = points2[i].x;
        two(1) = points2[i].y;
        points3D.push_back(triangulate(projection1, projection2, one, two));
    }
    // triangulate(projection1, projection2, points1, points2);
    
    // cv::sfm::triangulatePoints(points2D, projections, points3D);

    cv::viz::Viz3d window;
    std::cout << " before viz " << std::endl;
    window.showWidget("coordinate", cv::viz::WCoordinateSystem());
    window.setBackgroundColor(cv::viz::Color::black());
    window.showWidget("points", cv::viz::WCloud(points3D, cv::viz::Color::green()));
    std::cout << " middle viz " << std::endl;
    // window.saveScreenshot("/Users/kobikelemen/Documents/programming/c++/open_test/frames/3dpoints_frames");
    window.spin();

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


std::vector<Image> Scene_graph::get_frames()
{
    return frames;
}


std::vector<cv::DMatch> Image::get_matches(Image img) 
{
    cv::BFMatcher matcher(cv::NORM_HAMMING2, true);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptor, img.descriptor, matches);
    return matches;
}




Image::Image(int frame_n,  std::vector<cv::KeyPoint> kp, cv::Mat desc, cv::Mat f)
{
    frame_no = frame_n;
    keypoints = kp;
    descriptor = desc;
    frame = f;

}

void Image::add_connection(int c, std::vector<cv::DMatch> m)
{
    Edge edge(c, m);
    connections.push_back(edge);
    // could maybe improve efficiency by adding edges in order 
    // so when searching for edge can use bianry search
}