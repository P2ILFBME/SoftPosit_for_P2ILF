#include "./read_image.cpp"
#include "./read_pcd.cpp"
#include <iostream>
#include <string>
#include <vector>
#include "./softposit/softposit.hpp"
#include "./softposit/softposit.cpp"

using namespace cv;

std::vector<bloody::point2di_type> project_3DPoints(std::vector<bloody::point3d_type> worldPts, arma::mat rot,  bloody::point3d_type trans, bloody::CamInfo_type caminfo) // try to project point cloud data to image
{   
    float f = 100.0;
    std::vector<bloody::point2di_type> imagePts;
    
    for(int i=0; i<worldPts.size();i++){
        auto w = arma::dot(rot.col(2).t(),worldPts[i])+trans(2);
        auto x = arma::dot(rot.col(0).t(),worldPts[i])+trans(0);
        auto y = arma::dot(rot.col(1).t(),worldPts[i])+trans(1);
        imagePts.push_back(bloody::point2di_type{f*x/w, f*y/w}+caminfo.center);
        //std::cout<<f*x/w <<"  "<<f*y/w<<std::endl;
    }
    return imagePts;
}

int main()
{
    vector<int> raw_imagePts;
    raw_imagePts = read_img_to_point();
    vector<float> raw_worldPts;
    raw_worldPts = read_pcd_from_json();
    
    std::vector<bloody::point2di_type> imagePts;
    std::vector<bloody::point3d_type> worldPts;
    
    for (uint i=0u; i<raw_imagePts.size(); i+=2){
    imagePts.push_back(bloody::point2di_type{raw_imagePts[i], raw_imagePts[i+1]});
    //cout<<raw_imagePts[i]<<" "<<raw_imagePts[i+1]<<endl;
  }
    for (uint i=0u; i<raw_worldPts.size(); i+=3){
    worldPts.push_back(bloody::point3d_type{raw_worldPts[i], raw_worldPts[i+1], raw_worldPts[i+2]});
  }

  bloody::Param_type param{ 2.0E-4, 10.0};
  bloody::CamInfo_type caminfo{100.0f, bloody::point2di_type{640, 360}};
    
  bloody::Pose_type initpose;
  Matx31f Rvec = Matx31f(0.44519691244340875, -0.1219739338674524,0.8870862802499846);
  Matx33f dst;
  Rodrigues(Rvec, dst, noArray());
  arma::mat R(3,3);
  for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
          R(i,j)=dst(i,j);
      }
  }
  initpose.rot = arma::mat("0.8231, 0.4452, -0.3525; -0.3343, -0.122, -0.9345; -0.4591, 0.8871, 0.0484").t();
  // std::cout<<initpose.rot<<std::endl;
  initpose.trans = bloody::point3d_type{136, -113, 167};
  
  std::vector<bloody::point2di_type> imagePts_projected = project_3DPoints(worldPts, initpose.rot.t(),  initpose.trans, caminfo);
  
  auto maybe_pose = softposit(
    imagePts_projected,
    worldPts,
    param,
    initpose,
    caminfo
    );

  if (maybe_pose){
    auto pose = std::get<0>(*maybe_pose);
    std::cout<<pose.rot<<std::endl;
    std::cout<<pose.trans<<std::endl;
  }else{
    std::cout<<"failed"<<std::endl;
  }

  return 0;
}
