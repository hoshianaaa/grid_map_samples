#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace grid_map;
using namespace ros;

int main(int argc, char** argv)
{

    init(argc, argv, "show_image");
    NodeHandle nodeHandle("~");
    Publisher publisher = nodeHandle.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    Mat img = imread("/home/hoshina/catkin_ws/src/grid_map_samples/src/image.png");
    //単色画像の生成(参考:http://opencv.jp/cookbook/opencv_mat.html の 「cv::vecを使う」
    //cv::Mat_<cv::Vec3b> img(300, 300, cv::Vec3b(0,200,0)); // 追加

    cv::Mat gray_img;
    cvtColor(img, gray_img,CV_RGB2GRAY);

    imshow("", gray_img );
    waitKey(1);

    GridMap map({"elevation"});
    GridMapCvConverter::initializeFromImage(gray_img, 0.01, map, Position(0,0));
    GridMapCvConverter::addLayerFromImage<unsigned short, 1>(gray_img, "elevation", map, 0.0, 1.0);
    map.setFrameId("map");
    
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map.getLength().x(),map.getLength().y(), map.getSize()(0), map.getSize()(1));

    ros::Rate loop_rate(10);
    while (ros::ok())
    {

      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(map, message);
      publisher.publish(message);

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
