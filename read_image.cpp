#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
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
