#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "./softposit/softposit.hpp"
using namespace cv;
using namespace std;
vector<int> read_img_to_point(){
    char imageName[] = "./toy_data/image.png";
    Mat M = imread("../toy_data/image.png", IMREAD_COLOR);
    if(M.empty()){
        std::cout<<"error!"<<std::endl;
    }
    vector<int> linepoints;
    //std::cout<<M_HSV.channels()<<std::endl;
    //cout<<maxVal<<endl;
    int num_count = 0;
    for(int i = 0; i<M.rows; i++)
    {
        
        uchar* p = M.ptr<uchar>(i);
        for(int k=0; k<M.cols; k++){
            Vec3b bgr = M.at<Vec3b>(i, k);
          
            if(bgr[2]>50){
            
            if(num_count%30==0){
                linepoints.push_back(i);
                linepoints.push_back(k);
            }
            num_count++;
            
            }
            
        }
    }
    cout<<linepoints.size()<<endl;    
    //imshow("image", M);
    //waitKey();
    
    return linepoints;
}

void show_projected_img(std::vector<bloody::point2di_type> imagePts_projected, arma::mat color_map){
    Mat img = cv::Mat::zeros(720, 1280, CV_8UC3);
    for(int i=0; i<imagePts_projected.size();i++){
        int x = int(imagePts_projected[i](0));
        int y = int(imagePts_projected[i](1));
        
        //std::cout<<x<<"   "<<y<<std::endl;
        img.at<Vec3b>(y,x)[int(color_map[i])] =255;
        
        img.at<Vec3b>(360,640)[0] = img.at<Vec3b>(360,640)[1] = img.at<Vec3b>(360,640)[2] = 255;
    }
    Mat img2;
    cv::resize(img,img2,cv::Size(3,3));
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    imshow("image", img);
    
    waitKey();
    return;
}
