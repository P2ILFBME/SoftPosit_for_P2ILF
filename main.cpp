#include "./read_image.cpp"
#include "./read_pcd.cpp"
#include <iostream>
#include <string>
#include <vector>
#include "./softposit/softposit.hpp"
#include "./softposit/softposit.cpp"

using namespace cv;



int main()
{
    // read 2D/3D points from files
    vector<int> raw_imagePts;
    raw_imagePts = read_img_to_point(); // read_image_file
    vector<float> raw_worldPts;
    raw_worldPts = read_pcd_from_json(); // read json file
    
    // transform points to arma type
    std::vector<bloody::point2di_type> imagePts;
    std::vector<bloody::point3d_type> worldPts;
    for (uint i=0u; i<raw_imagePts.size(); i+=2)
    {
        imagePts.push_back(bloody::point2di_type{1280-raw_imagePts[i+1], raw_imagePts[i]});      // get image points in (x, y) format
        //cout<<raw_imagePts[i]<<" "<<raw_imagePts[i+1]<<endl;
    }
    for (uint i=0u; i<raw_worldPts.size(); i+=3)
    {
        worldPts.push_back(bloody::point3d_type{raw_worldPts[i], raw_worldPts[i+1], raw_worldPts[i+2]});    // get world 3D points
    }
    // fit image points to curve and resample
  cv::Mat A;
  std::vector<bloody::point2di_type> fitImagePts;
  FitPolynomialCurve(imagePts, fitImagePts, 4,  A);
  // set inner parameter and outer parameter and softposit parameter
  bloody::Param_type param{ 0.00004, 10.0}; //0.001
  bloody::CamInfo_type caminfo{500.0f, bloody::point2di_type{640, 360}}; // inner parameter of camera
  
  // set init pose change
  Matx31f Rvec = Matx31f(0.3, 0.1, 0.2);
  bloody::point3d_type trans = bloody::point3d_type{40, -20.39718, 30.29930};
  Matx33f dst;
  Rodrigues(Rvec, dst, noArray());
  arma::mat R_change(3,3);
  for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
          R_change(i,j)=dst(i,j);
      }
  }
  arma::mat rot1 = arma::mat("0.82311, -0.33426, -0.45906;0.44522, -0.122, 0.88705;-0.35258, -0.93459, 0.048413");
  arma::mat rot2 = arma::mat("0.8231, 0.4452, -0.3525, 136.5396; -0.3343, -0.122, -0.9345, -113.6996; -0.4591, 0.8871, 0.0484, 167.6742; 0,0,0,1");
  rot2 = arma::inv(rot2);
  //std::cout<<rot2<<std::endl;
  
  // set gt pose
  bloody::Pose_type initPose, gtPose;
  gtPose.rot = rot2(arma::span(0,2), arma::span(0,2));//arma::mat("0.82311, -0.33426, -0.45906;0.44522, -0.122, 0.88705;-0.35258, -0.93459, 0.048413");//("0.8231, 0.4452, -0.3525; -0.3343, -0.122, -0.9345; -0.4591, 0.8871, 0.0484");
  gtPose.trans = bloody::point3d_type{rot2(0,3), rot2(1,3), rot2(2,3)};//{-73.420, -223.39718, -66.29930}
  std::vector<bloody::point2di_type> imagePts_projected=imagePts; //imagePts
  //imagePts_projected = project_3DPoints(worldPts,imagePts_projected,  gt_pose.rot,  gt_pose.trans, caminfo);   // project worldPts to 2D imagePts_projected(at read_pcd.cpp)
  
  
  //---------------------------------------------------------create init pose----------------------------------------------------------------
  int nImagePts = imagePts_projected.size();
  initPose.rot = arma::mat("0.82311, -0.33426, -0.45906; 0.44522, -0.122, 0.88705;-0.35258, -0.93459, 0.048413");
  initPose.rot = R_change*initPose.rot;
  
  initPose.trans = bloody::point3d_type{-73.420, -223.39718, -66.29930}+trans;
  //imagePts_projected = project_3DPoints(worldPts, imagePts_projected,  initpose.rot,  initpose.trans, caminfo);
  
  arma::mat imageOnes = arma::ones<arma::mat>(nImagePts, 1)*2;
  arma::mat color_map = arma::join_rows(color_map, imageOnes);
  show_projected_img(imagePts_projected, color_map, true);
  

  //begin softposit
  try{
    auto maybe_pose = softposit(
        imagePts_projected,
        worldPts,
        param,
        initPose,
        caminfo
        );
    if (maybe_pose){
        auto pose = std::get<0>(*maybe_pose);
        std::cout<<pose.rot<<std::endl;
        std::cout<<pose.trans<<std::endl;
        int size1 = imagePts_projected.size();
        imagePts_projected = project_3DPoints(worldPts, imagePts_projected,  initPose.rot,  initPose.trans, caminfo);
        imageOnes = arma::ones<arma::mat>(imagePts_projected.size()-size1, 1)*1;
        color_map = arma::join_cols(color_map, imageOnes);
    
        size1 = imagePts_projected.size();
        imagePts_projected = project_3DPoints(worldPts, imagePts_projected, pose.rot,  pose.trans, caminfo);
        imageOnes = arma::ones<arma::mat>(imagePts_projected.size()-size1, 1)*0;
        color_map = arma::join_cols(color_map, imageOnes);
        show_projected_img(imagePts_projected, color_map, true);
  }
    else
    {
        std::cout<<"failed"<<std::endl;
    }
  }
  catch(runtime_error error)
  {
    int size1 = imagePts_projected.size();
    imagePts_projected = project_3DPoints(worldPts, imagePts_projected, initPose.rot, initPose.trans, caminfo);
    imageOnes = arma::ones<arma::mat>(imagePts_projected.size()-size1, 1)*1;
    color_map = arma::join_cols(color_map, imageOnes);
    show_projected_img(imagePts_projected, color_map, true);
    
  }
  // show result

  return 0;
}
