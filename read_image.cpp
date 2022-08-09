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
    int min_x = 100000;
    int max_x = -1;
    for(int i = 0; i<M.rows; i++)
    {
        
        uchar* p = M.ptr<uchar>(i);
        for(int k=0; k<M.cols; k++){
            Vec3b bgr = M.at<Vec3b>(i, k);
          
            if(bgr[2]>50){
            
            if(num_count%1==0){
                linepoints.push_back(i);
                linepoints.push_back(k);
            }
            num_count++;
            if(k<min_x){
                min_x = k;
            }
            if(k>max_x){
                max_x = k;
            }
            
            }
            
        }
    }
    cout<<linepoints.size()<<endl;
    //cout<<"min_x:  "<<min_x<<endl;
    //cout<<"max_x:  "<<max_x<<endl;
    //imshow("image", M);
    //waitKey();
    
    return linepoints;
}


void FitPolynomialCurve(std::vector<bloody::point2di_type>& points,std::vector<bloody::point2di_type>& out_points,  int n, cv::Mat& A){
    //最小二乘法多项式曲线拟合原理与实现 https://blog.csdn.net/jairuschan/article/details/7517773/
    //https://www.cnblogs.com/fengliu-/p/8031406.html
    int N = points.size();
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++){
        for (int j = 0; j < n + 1; j++){
            for (int k = 0; k < N; k++){
                X.at<double>(i, j) = X.at<double>(i, j) +
                        std::pow(points[k](0), i + j);
            }
        }
    }
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++){
        for (int k = 0; k < N; k++){
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                    std::pow(points[k](0), i) * points[k](1);
        }
    }
    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    cv::solve(X, Y, A, cv::DECOMP_LU);
    
    for (int x = 1280-680; x < 1280-447; x=x+5)
    {
        double y=A.at<double>(0, 0);
        for(int k=1;k<=A.rows;k++){
            y += A.at<double>(k, 0)*std::pow(x, k);
        }
        out_points.push_back(bloody::point2di_type{x, int(y)});
    }
    std::cout<<out_points.size()<<std::endl;
}


void show_projected_img(std::vector<bloody::point2di_type> imagePts_projected, arma::mat color_map, bool wait = false){
    Mat img = Mat(720, 1280, CV_8UC3,Scalar(255,255,255));
    for(int i=0; i<imagePts_projected.size();i++){
        int x = int(imagePts_projected[i](0));
        int y = int(imagePts_projected[i](1));
        
        //std::cout<<x<<"   "<<y<<std::endl;
        img.at<Vec3b>(y,x)[0]=img.at<Vec3b>(y,x)[1]=img.at<Vec3b>(y,x)[2]=0;
        img.at<Vec3b>(y,x)[int(color_map[i])] =255;
        
        img.at<Vec3b>(360,640)[0] = img.at<Vec3b>(360,640)[1] = img.at<Vec3b>(360,640)[2] = 255;
    }
    Mat img2;
    cv::resize(img,img2,cv::Size(3,3));
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::resizeWindow("image", 3840,2560);
    imshow("image", img);
    if (wait){
        waitKey(0);
    }
    else{
        waitKey(1);
    }
    
    return;
}
