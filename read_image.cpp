#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "./softposit/softposit.hpp"

using namespace cv;
using namespace std;
extern int IMAGE_HEIGHT;
extern int IMAGE_WIDTH;
vector<int> read_img_to_point(string imageName = "./toy_data/image.png"){
    
    Mat M = imread(imageName, IMREAD_COLOR);
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
            
            if(num_count%10==0){
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
    
    return linepoints;
}


vector<int> read_img_to_point2(string imageName = "./toy_data/image.png"){
    
    Mat M = imread(imageName, IMREAD_COLOR);
    if(M.empty()){
        std::cout<<"error!"<<std::endl;
    }
    vector<int> linepoints;
    //std::cout<<M_HSV.channels()<<std::endl;
    //cout<<maxVal<<endl;
    int num_count = 0;
    int min_x = 100000;
    int max_x = -1;
    //imshow("image", M);
    //waitKey();
    for(int i = 0; i<M.rows; i++)
    {
        
        uchar* p = M.ptr<uchar>(i);
        for(int k=0; k<M.cols; k++){
            Vec3b bgr = M.at<Vec3b>(i, k);
          
            if(bgr[0]>220 && bgr[1]<200 &&bgr[2]<100){
            
            if(num_count%10==0){
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
            else{
                M.at<Vec3b>(i, k)[0] = M.at<Vec3b>(i, k)[1] = M.at<Vec3b>(i, k)[2]=0;
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
    int xMin = 20000;
    int xMax = 0;
    for (int k = 0; k < N; k++){
        if(points[k](0)<xMin) xMin = points[k](0);
        if(points[k](0)>xMax) xMax = points[k](0);
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
    
    for (int x = xMin; x < xMax; x=x+1)
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
    Mat img = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3,Scalar(255,255,255));
    for(int i=0; i<imagePts_projected.size();i++){
        int x = int(imagePts_projected[i](0));
        int y = int(imagePts_projected[i](1));
        
        //std::cout<<x<<"   "<<y<<std::endl;
        img.at<Vec3b>(y,x)[0]=img.at<Vec3b>(y,x)[1]=img.at<Vec3b>(y,x)[2]=0;
        img.at<Vec3b>(y,x)[int(color_map[i])] =255;
        
        img.at<Vec3b>(IMAGE_HEIGHT/2,IMAGE_WIDTH/2)[0] = img.at<Vec3b>(IMAGE_HEIGHT/2,IMAGE_WIDTH/2)[1] = img.at<Vec3b>(IMAGE_HEIGHT/2,IMAGE_WIDTH/2)[2] = 255;
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

double calculate_shape_contour(std::vector<bloody::point2di_type> image1, std::vector<bloody::point2di_type> image2){
    vector<cv::Point> contour1, contour2;
    for(int i=0;i<image1.size();i++){
        cv::Point point;
        point.x = float(image1[i](0));
        point.y = float(image1[i](1));
        contour1.push_back(point);
    }

    vector<vector<cv::Point>> finalContour1, finalContour2;
    //finalContour1.push_back(contour1);
    for(int i=0;i<image2.size();i++){
        cv::Point point;
        point.x = float(image2[i](0));
        point.y = float(image2[i](1));
        contour2.push_back(point);
    }
    
    //finalContour2.push_back(contour2);
    cout<<contour1.size()<<endl;
    cout<<contour2.size()<<endl;
    double result=cv::matchShapes(contour1,contour2,cv::CONTOURS_MATCH_I3,0);
    cout<<"ShapeMatch:"<<result<<endl;
    return result;
}

void show_projected_img_with_corr(std::vector<bloody::point2di_type> imagePts_projected, arma::mat color_map, std::vector<bloody::point2di_type> corresspondence, int size1,  bool wait = false){
    Mat img = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3,Scalar(255,255,255));
    for(int i=0; i<imagePts_projected.size();i++){
        int x = int(imagePts_projected[i](0));
        int y = int(imagePts_projected[i](1));
        
        //std::cout<<x<<"   "<<y<<std::endl;
        img.at<Vec3b>(y,x)[0]=img.at<Vec3b>(y,x)[1]=img.at<Vec3b>(y,x)[2]=0;
        img.at<Vec3b>(y,x)[int(color_map[i])] =255;
        
        img.at<Vec3b>(IMAGE_HEIGHT/2,IMAGE_WIDTH/2)[0] = img.at<Vec3b>(IMAGE_HEIGHT/2,IMAGE_WIDTH/2)[1] = img.at<Vec3b>(IMAGE_HEIGHT/2,IMAGE_WIDTH/2)[2] = 255;
    }
    for(int k=0;k<corresspondence.size();k++){
        int idxImag = int(corresspondence[k](0));
        int idxWorld = int(corresspondence[k](1))+size1;
        
        int x1 = int(imagePts_projected[idxImag](0));
        int y1 = int(imagePts_projected[idxImag](1));
        
        int x2 = int(imagePts_projected[idxWorld](0));
        int y2 = int(imagePts_projected[idxWorld](1));
        line(img, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 1, CV_AA);
        
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

