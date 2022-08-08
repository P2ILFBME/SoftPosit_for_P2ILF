#include <iostream>
#include <string>
#include <json/json.h>
#include <fstream>
#include <pcl-1.10/pcl/common/common_headers.h>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>
#include <pcl-1.10/pcl/console/parse.h>
#include "./softposit/softposit.hpp"

using namespace std;

vector<float> read_pcd_from_json(){
    Json::Reader reader;
    Json::Value root;
    vector<float> worldPts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    ifstream srcFile("../toy_data/registration.json", ios::binary);
    if (!srcFile.is_open()){
        cout<<"Fail to open src.json"<<endl;
    }
    if (! reader.parse(srcFile, root)){
        cout<<root.size()<<endl;
        point_cloud_ptr->width = 1;
        point_cloud_ptr->height= root["scene_points"].size();
        point_cloud_ptr->is_dense = false;
        point_cloud_ptr->points.resize(point_cloud_ptr->width*point_cloud_ptr->height); 
        for (int i=0; i<root["scene_points"].size();i++){
            pcl::PointXYZ point;
            worldPts.push_back(root["scene_points"][i][0].asFloat());
            worldPts.push_back(root["scene_points"][i][1].asFloat());
            worldPts.push_back(root["scene_points"][i][2].asFloat());
            
            point_cloud_ptr->points[i].x = root["scene_points"][i][0].asFloat();
            point_cloud_ptr->points[i].y = root["scene_points"][i][1].asFloat();
            point_cloud_ptr->points[i].z = root["scene_points"][i][2].asFloat();
            //point_cloud_ptr->points.push_back (point);
        }
    }
    cout<<worldPts.size()<<endl;
    pcl::io::savePCDFile("../toy.pcd", *point_cloud_ptr);
    system("pause");
    return worldPts;
}

std::vector<bloody::point2di_type> project_3DPoints(std::vector<bloody::point3d_type> worldPts, std::vector<bloody::point2di_type> older_image_pts, arma::mat rot,  bloody::point3d_type trans, bloody::CamInfo_type caminfo) // try to project point cloud data to image
{   
    float f = caminfo.focalLength;
    std::vector<bloody::point2di_type> imagePts=older_image_pts;
    
    for(int i=0; i<worldPts.size();i++){
        //std::cout<<worldPts[i]<<"//"<<rot.row(2)<<std::endl;
        auto w = arma::dot(rot.row(2),worldPts[i])+trans(2);
        auto x = arma::dot(rot.row(0),worldPts[i])+trans(0);
        auto y = arma::dot(rot.row(1),worldPts[i])+trans(1);
        if(((f*x/w+caminfo.center(0))<1280)&&((f*x/w+caminfo.center(0)))>0&&((f*y/w+caminfo.center(1)))>0&&((f*y/w+caminfo.center(1)))<720){
        
            imagePts.push_back((bloody::point2di_type{f*x/w, f*y/w}+caminfo.center));
        }
        // std::cout<<x<<"   "<<y<<"   "<<w<<std::endl;
        
    }
    // std::cout<<worldPts.size()<<std::endl;
    return imagePts;
}
