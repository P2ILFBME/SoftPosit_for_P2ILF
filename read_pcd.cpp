#include <iostream>
#include <string>
#include <json/json.h>
#include <fstream>
#include <pcl-1.10/pcl/common/common_headers.h>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>
#include <pcl-1.10/pcl/console/parse.h>


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
