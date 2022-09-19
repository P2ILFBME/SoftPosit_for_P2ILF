#include <iostream>
#include <string>
#include <json/json.h>
#include <fstream>
#include <pcl-1.10/pcl/common/common_headers.h>
#include <pcl-1.10/pcl/io/ply_io.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>
#include <pcl-1.10/pcl/console/parse.h>
#include "./softposit/softposit.hpp"
#include<algorithm>

extern int IMAGE_HEIGHT;
extern int IMAGE_WIDTH;
using namespace std;

vector<float> read_pcd_from_json(string jsonPath="../toy_data/registration.json"){
    Json::Reader reader;
    Json::Value root;
    vector<float> worldPts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    float xmin=1000,ymin=1000,zmin=1000,xmax=0,ymax=0,zmax=0;
    ifstream srcFile(jsonPath, ios::binary);
    if (!srcFile.is_open()){
        cout<<"Fail to open src.json"<<endl;
    }
    reader.parse(srcFile, root);
    if (1){
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
            xmin = min(xmin, point_cloud_ptr->points[i].x);
            ymin = min(ymin, point_cloud_ptr->points[i].y);
            zmin = min(zmin, point_cloud_ptr->points[i].z);
            xmax = max(xmax, point_cloud_ptr->points[i].x);
            ymax = max(ymax, point_cloud_ptr->points[i].y);
            zmax = max(zmax, point_cloud_ptr->points[i].z);
        }
    }
    cout<<xmin<<" "<<ymin<<" "<<zmin<<endl;
    cout<<xmax<<" "<<ymax<<" "<<zmax<<endl;
    //pcl::io::savePCDFile("../toy.pcd", *point_cloud_ptr);
    //system("pause");
    return worldPts;
}
vector<float> read_pcd_from_ply(string plyPath="../try_data/Liver_sect.ply"){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vector<float> worldPts;
    if(pcl::io::loadPLYFile<pcl::PointXYZ>(plyPath, *cloud)==-1){
        cout<<"could not read file."<<endl;
        system("pause");
        return worldPts;
    }
    for(auto point:cloud->points){
        worldPts.push_back(point.x);
        worldPts.push_back(point.y);
        worldPts.push_back(point.z);
    }
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
        if(((f*x/w+caminfo.center(0))<IMAGE_WIDTH)&&((f*x/w+caminfo.center(0)))>0&&((f*y/w+caminfo.center(1)))>0&&((f*y/w+caminfo.center(1)))<IMAGE_HEIGHT){
        
            imagePts.push_back((bloody::point2di_type{f*x/w, f*y/w}+caminfo.center));
        }
        // std::cout<<x<<"   "<<y<<"   "<<w<<std::endl;
        
    }
    // std::cout<<worldPts.size()<<std::endl;
    return imagePts;
}
