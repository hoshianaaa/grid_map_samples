#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, const char * argv[])
{
    //Mat image = imread( "image.jpg" );
    //単色画像の生成(参考:http://opencv.jp/cookbook/opencv_mat.html の 「cv::vecを使う」
    cv::Mat_<cv::Vec3b> img(300, 300, cv::Vec3b(0,200,0)); // 追加

    imshow("", img );
    waitKey(0);

    return 0;
}
