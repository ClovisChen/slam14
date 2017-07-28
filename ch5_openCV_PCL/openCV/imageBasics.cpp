#include <iostream>
#include <chrono>
using namespace std;
//using namespace cv;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    //read
    cv::Mat image;
    image=cv::imread ( argv[1]);
    //judge
    if (image.data ==nullptr)
    {
        cerr<<"file"<<argv[1]<<"do not exist"<<endl;
        return 0;
    }

    //file was read correct
    cout<<"width is "<<image.cols<<"height is"<<image.rows<<"channel is "<<image.channels()<<endl;
    cv::namedWindow("image",0);
    cv::imshow("image", image);
    cv::waitKey(0);//pause to wait for an input
    //image type
    if (image.type() !=CV_8UC1 && image.type() !=CV_8UC3)
    {
        cout<<"please input an color or gray image. "<<endl;
        return 0;
    }

    //traverse the image
    //timing (use std::chrono)
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y=0; y<image.rows; y++)
    {
        for(size_t x=0; x<image.cols;x++)
        {
            unsigned char* row_ptr =image.ptr<unsigned char> (y);
            unsigned char* data_ptr =&row_ptr[ x*image.channels() ];//output the pixel's channel number
            for (int c=0 ; c != image.channels(); c++)
            {
                unsigned char data =data_ptr[c];
                //data is the value of the cth channel
            }
        }

    }

    chrono::steady_clock::time_point t2=chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"traverse time: "<<time_used.count()<<" s"<<endl;

    //copy about cv::Mat
    //direct assignment can not copy
    //modify image_another will change image
    cv::Mat image_another = image;
    image_another (cv::Rect(0,0,100,100)).setTo(255);
    cv::namedWindow("image",0);
    cv::imshow("image", image);
    cv::waitKey(0);

    //clone function to copy
    cv::Mat image_clone = image.clone();
    image_clone (cv::Rect(0,0,100,100)).setTo(0);
    cv::namedWindow("image",0);
    cv::imshow ("image", image);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}