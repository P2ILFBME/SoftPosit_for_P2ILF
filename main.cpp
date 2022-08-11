#include "./read_image.cpp"
#include "./read_pcd.cpp"
#include <iostream>
#include <string>
#include <vector>
#include "./softposit/softposit.hpp"
#include "./softposit/softposit.cpp"
#include "./curve_generator.cpp"
#include <stdlib.h> 
#include <cmath>
using namespace cv;
using namespace std;

string JSON_PATH = "../toy_data/3/registration.json";
string IMAGE_PATH = "../toy_data/3/label.png";

void get_worldPts(std::vector<bloody::point3d_type>& worldPts, bool bezier_key)
{
    vector<float> raw_worldPts;
    if (bezier_key){
        raw_worldPts = rrtAlgorithm(0, 0, 0, 8, 8, 8, 1, 2+random()%7, 2+random()%7, 2+random()%7, 0.5, 0.5, 5000);
    }
    else{
        raw_worldPts = read_pcd_from_json(JSON_PATH); // read json file
    }
    
    for (uint i=0u; i<raw_worldPts.size(); i+=3)
    {
        worldPts.push_back(bloody::point3d_type{raw_worldPts[i], raw_worldPts[i+1], raw_worldPts[i+2]});    // get world 3D points
    }
}

void get_imagePts_projected(std::vector<bloody::point2di_type> &imagePts_projected, std::vector<bloody::point3d_type>& worldPts, bloody::CamInfo_type caminfo, bloody::Pose_type gtPose, bool curveFit=true, bool load_picture=false)
{
    std::vector<bloody::point2di_type> fitImagePts;
    if(load_picture)
    {
        vector<int> raw_imagePts;
        raw_imagePts = read_img_to_point(IMAGE_PATH);
        for (uint i=0u; i<raw_imagePts.size(); i+=2)
        {
            imagePts_projected.push_back(bloody::point2di_type{1280-raw_imagePts[i+1], raw_imagePts[i]});      // get image points in (x, y) format
            //cout<<raw_imagePts[i]<<" "<<raw_imagePts[i+1]<<endl;
        }
    }
    
    else{
        imagePts_projected = project_3DPoints(worldPts,imagePts_projected,  gtPose.rot,  gtPose.trans, caminfo);
    }
    if (curveFit)
        {
           cv::Mat A;
           FitPolynomialCurve(imagePts_projected, fitImagePts, 4,  A); 
           imagePts_projected = fitImagePts;
        }
    
}

void get_noised_pose(bloody::Pose_type& initPose, bloody::Pose_type& gtPose, Matx31f Rvec, bloody::point3d_type trans)
{
    Matx33f dst;
    Rodrigues(Rvec, dst, noArray());
    arma::mat R_change(3,3);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            R_change(i,j)=dst(i,j);
        }
    }
    initPose.rot = R_change*gtPose.rot;
    initPose.trans = gtPose.trans+trans;
}

bool evaluation(bloody::Pose_type& initPose, bloody::Pose_type& gtPose, double r_threshold, double t_threshold)
{
    double rte;
    rte = arma::sum(arma::square(initPose.trans-gtPose.trans));
    cout<<sqrt(rte)<<endl;
    if (sqrt(rte)<10){
        return true;
    }
    return false;
}

void load_gt_from_json(string jsonPath, bloody::Pose_type& gtPose)
{
    Json::Reader reader;
    Json::Value root;
    ifstream srcFile(jsonPath, ios::binary);
    if (!srcFile.is_open()){
        cout<<"Fail to open src.json"<<endl;
    }
    arma::mat rot2 = arma::eye(4,4);
    reader.parse(srcFile, root);
    if (1){
        for (int i=0; i<root["extrinsics"].size();i++){
            rot2(i,0) = root["extrinsics"][i][0].asFloat();
            rot2(i,1) = root["extrinsics"][i][1].asFloat();
            rot2(i,2) = root["extrinsics"][i][2].asFloat();
            rot2(i,3) = root["extrinsics"][i][3].asFloat();
        }
    }
    
    rot2 = arma::inv(rot2);
    cout<<rot2<<endl;
    gtPose.rot = rot2(arma::span(0,2), arma::span(0,2));
    gtPose.trans = bloody::point3d_type{rot2(0,3), rot2(1,3), rot2(2,3)};
}

int run()
{   
    std::vector<bloody::point2di_type> imagePts;
    std::vector<bloody::point3d_type> worldPts;
    get_worldPts(worldPts, false);  // bezier key
    std::vector<bloody::point2di_type> imagePts_projected;
    bool result;
    bloody::Param_type param{ 0.00004, 10.0}; //0.001
    bloody::CamInfo_type caminfo{500.0f, bloody::point2di_type{640, 360}}; // inner parameter of camera
    arma::mat rot2 = arma::mat("0.8231, 0.4452, -0.3525, 136.5396; -0.3343, -0.122, -0.9345, -113.6996; -0.4591, 0.8871, 0.0484, 167.6742; 0,0,0,1");
    rot2 = arma::inv(rot2);
    bloody::Pose_type initPose, gtPose;
    bool randomGt=false;
    do{
        imagePts_projected.clear();
        if(randomGt){
            gtPose.rot = rot2(arma::span(0,2), arma::span(0,2));//arma::mat("0.82311, -0.33426, -0.45906;0.44522, -0.122, 0.88705;-0.35258, -0.93459, 0.048413");//("0.8231, 0.4452, -0.3525; -0.3343, -0.122, -0.9345; -0.4591, 0.8871, 0.0484");
            gtPose.trans = bloody::point3d_type{rot2(0,3), rot2(1,3), rot2(2,3)};//{-73.420, -223.39718, -66.29930}
            // get_noised_pose(gtPose, gtPose, Matx31f(-0.6+(rand()%14)/10.0, -0.3+(rand()%6)/10.0, -0.5+(rand()%10)/10.0), bloody::point3d_type{0, 0, 0});   //(rand()%7)/10.0
        }
        else{
            load_gt_from_json(JSON_PATH, gtPose);
        }
        get_imagePts_projected(imagePts_projected, worldPts, caminfo, gtPose, false, true); 
    }
    while(imagePts_projected.size()==0);
    Matx31f Rvec = Matx31f(-3+rand()%6, -3+rand()%6, -3+rand()%6);      // -3+rand()%6, -3+rand()%6, -3+rand()%6
    bloody::point3d_type trans = bloody::point3d_type{double(-900+rand()%1800),double(-300+rand()%600),-1300};   // double(-900+rand()%1800),double(-300+rand()%600),-1300
    get_noised_pose(initPose, gtPose, Rvec, trans);
    
    // transform points to arma type
    
    
  
    //---------------------------------------------------------create init pose----------------------------------------------------------------
  
    // imagePts_projected = project_3DPoints(worldPts, imagePts_projected,  initPose.rot,  initPose.trans, caminfo);
    std::vector<bloody::point2di_type> initProject;
    initProject = project_3DPoints(worldPts, imagePts_projected,  gtPose.rot,  gtPose.trans, caminfo);
    int nImagePts = initProject.size();
    arma::mat imageOnes = arma::ones<arma::mat>(nImagePts, 1)*2;
    arma::mat color_map = arma::join_rows(color_map, imageOnes);
    show_projected_img(initProject, color_map, true);
    

  //begin softposit
    bloody::Param_type param2 = param;
    for(int i =0;i<10;i++){
        try{
            bloody::Pose_type pose;
            auto maybe_pose = softposit(
                imagePts_projected,
                worldPts,
                param2,
                initPose,
                caminfo
                );
            if (maybe_pose){
                pose = std::get<0>(*maybe_pose);
                std::cout<<pose.rot<<std::endl;
                std::cout<<pose.trans<<std::endl;
                //int size1 = imagePts_projected.size();
                //color_map = arma::ones<arma::mat>(size1, 1)*2;
                //imagePts_projected = project_3DPoints(worldPts, imagePts_projected,  initPose.rot,  initPose.trans, caminfo);
                //imageOnes = arma::ones<arma::mat>(imagePts_projected.size()-size1, 1)*1;
                color_map = arma::join_cols(color_map, imageOnes);

                //imagePts_projected = project_3DPoints(worldPts, imagePts_projected, pose.rot,  pose.trans, caminfo);
                //imageOnes = arma::ones<arma::mat>(imagePts_projected.size()-size1, 1)*0;
                //color_map = arma::join_cols(color_map, imageOnes);
                //show_projected_img(imagePts_projected, color_map, false);
                result = evaluation(pose,gtPose,1,1);
                if(result) return 1;
                else {
                    Matx31f Rvec = Matx31f(-3+rand()*6, -3+rand()*6, -3+rand()*6);
                    bloody::point3d_type trans = bloody::point3d_type{double(-900+rand()%1800),double(-300+rand()%600),-1300};   //-500,-309,-1300
                    get_noised_pose(initPose, gtPose, Rvec, trans);
                    param2 = param;
                }
        }
            else
            {
                std::cout<<"failed"<<std::endl;
                Matx31f Rvec = Matx31f(-3+rand()*6, -3+rand()*6, -3+rand()*6);
                bloody::point3d_type trans = bloody::point3d_type{double(-900+rand()%1800),double(-300+rand()%600),-1300};   //-500,-309,-1300
                get_noised_pose(initPose, gtPose, Rvec, trans);
                param2 = param;
                
            }
    }
        catch(runtime_error error)
        {
            int size1 = imagePts_projected.size();
            imagePts_projected = project_3DPoints(worldPts, imagePts_projected, initPose.rot, initPose.trans, caminfo);
            imageOnes = arma::ones<arma::mat>(imagePts_projected.size()-size1, 1)*1;
            color_map = arma::join_cols(color_map, imageOnes);
            //show_projected_img(imagePts_projected, color_map);
            Matx31f Rvec = Matx31f(-3+rand()*6, -3+rand()*6, -3+rand()*6);
            bloody::point3d_type trans = bloody::point3d_type{double(-900+rand()%1800),double(-300+rand()%600),-1300};   //-500,-309,-1300
            get_noised_pose(initPose, gtPose, Rvec, trans);
            param2 = param;
        }
        
        imagePts_projected.clear();
        get_imagePts_projected(imagePts_projected, worldPts, caminfo, gtPose, false, false);
        
    }
  // show result
    return 0;
}

int main()
{
    double numRun = 1;
    double valid_num = numRun;
    double success_num = 0;
    for(int i=0; i<numRun;i++){
        try{
            success_num += run();
        }
        catch(logic_error error){
            valid_num -=1;
            continue;
        }
    }
    std::cout<<success_num/valid_num<<std::endl;
    return 0;
}
