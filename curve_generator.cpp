//
// Created by Nikhil Italiya on 20-12-2019.
//
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <fstream>
#include <time.h>


using namespace std;
//node for storing values and parent information
struct theNode {
    double x = 0;
    double y = 0;
    double z = 0;
    int parent = 0;
};

//creates node with given set of xyz values and the name of parent node as previous node
theNode* createNode(double x, double y, double z, int parent) {
    auto* temp = new theNode;
    temp->x = x;
    temp->y = y;
    temp->z = z;
    temp->parent = parent;
    return temp;
}

//to find the nearest node
std::pair <int, theNode>  FindNearest(double xrand, double yrand, double zrand, std::vector<theNode> tree) {
    std::vector<double > disvec;
    //loop for creating a vector of distance from the random point to all the nodes
    for (auto temp : tree) {
        double distance = ((xrand - temp.x) * (xrand - temp.x)) + ((yrand - temp.y) * (yrand - temp.y)) +
                          ((zrand - temp.z) * (zrand - temp.z));
        double dd = sqrt(distance);
        disvec.push_back(dd);
    }
    //finds the mimimum value
    __gnu_cxx::__normal_iterator<double*, vector<double>> leastIterator = std::min_element(disvec.begin(), disvec.end());
    auto nearest = (double)*leastIterator;


    // a copy of the nearest node
    int parentindex = 0;
    theNode tempNearNode;
    // loop to get the index of the parent node
    for (int i = 0; i < disvec.size(); ++i) {
        if (disvec.at(i) == nearest) {
            tempNearNode = tree.at(i);
            parentindex = i;
            break;
        }
    }

    return std::make_pair(parentindex, tempNearNode);
}

// to print the tree
void printTree(std::vector<theNode> tree1, int j) {
    double xvalue[j];
    double yvalue[j];
    double zvalue[j];
    int pvalue[j];


    //print the tree
    for (int k = 0; k < tree1.size(); ++k) {
        xvalue[k] = tree1.at(k).x;
        yvalue[k] = tree1.at(k).y;
        zvalue[k] = tree1.at(k).z;
        pvalue[k] = tree1.at(k).parent;
    }
    cout << "arrx = [";
    for (int l = 0; l < tree1.size(); ++l) {
        if (l == tree1.size() - 1) {
            cout << xvalue[l];
        }
        else {
            cout << xvalue[l] << ", " << endl;
        }
    }
    cout << "]" << endl;
    cout << "arry = [";
    for (int l = 0; l < tree1.size(); ++l) {
        if (l == tree1.size() - 1) {
            cout << yvalue[l];
        }
        else {
            cout << yvalue[l] << "," << endl;
        }
    }
    cout << "]" << endl;
    cout << "arrz = [";
    for (int l = 0; l < tree1.size(); ++l) {
        if (l == tree1.size() - 1) {
            cout << zvalue[l];
        }
        else {
            cout << zvalue[l] << "," << endl;
        }
    }
    cout << "]" << endl;
}

//to show the tree
void ShowTree(std::vector<theNode> tree1, int j) {
    double xvalue[j];
    double yvalue[j];
    double zvalue[j];
    int pvalue[j];

    //建立可视化窗口
    for (int k = 0; k < tree1.size(); ++k) {
        xvalue[k] = tree1.at(k).x;
        yvalue[k] = tree1.at(k).y;
        zvalue[k] = tree1.at(k).z;
        pvalue[k] = tree1.at(k).parent;
       
    }
}


vector<float> bezier_points(std::vector<theNode> tree1, int num_points, int j)
{
    //建立可视化窗口
    double xvalue[j];
    double yvalue[j];
    double zvalue[j];
    int pvalue[j];
    vector<float> worldPts;

    for (int k = 0; k < tree1.size(); ++k) {
        xvalue[k] = tree1.at(k).x;
        yvalue[k] = tree1.at(k).y;
        zvalue[k] = tree1.at(k).z;
        pvalue[k] = tree1.at(k).parent;
//         if(k == tree1.size() - 1) std::cout<<"y"<<xvalue[k]<<" "<< yvalue[k] <<" "<< yvalue[k] << std::endl;
    }
    
    //get control points
//     double x0, y0, z0, x1, y1, z1;
    double x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3;
    double tc = 0.5;
    int tree_size = tree1.size() * 2;
    double cx[tree_size], cy[tree_size], cz[tree_size];
    for(int i = 0; i < tree1.size(); i++){
        if(i == 0){
            int k = i;
            x0 = tree1.at(k).x;
            y0 = tree1.at(k).y;
            z0 = tree1.at(k).z;
            x1 = tree1.at(k).x;
            y1 = tree1.at(k).y;
            z1 = tree1.at(k++).z;
            x2 = tree1.at(k).x;
            y2 = tree1.at(k).y;
            z2 = tree1.at(k).z;
        }
        else if(i == tree1.size() - 1){
            int k = i - 1;
            x0 = tree1.at(k).x;
            y0 = tree1.at(k).y;
            z0 = tree1.at(k++).z;
            x1 = tree1.at(k).x;
            y1 = tree1.at(k).y;
            z1 = tree1.at(k).z;
            x2 = tree1.at(k).x;
            y2 = tree1.at(k).y;
            z2 = tree1.at(k).z;
        }
        else{
            int k = i - 1;
            x0 = tree1.at(k).x;
            y0 = tree1.at(k).y;
            z0 = tree1.at(k++).z;
            x1 = tree1.at(k).x;
            y1 = tree1.at(k).y;
            z1 = tree1.at(k++).z;
            x2 = tree1.at(k).x;
            y2 = tree1.at(k).y;
            z2 = tree1.at(k).z;
        }
        
        double d01 = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2));
        double d12 = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));

        double fa = tc * d01 / (d01 + d12);
        double fb = tc * d12 / (d01 + d12);
        
        int c_num = i * 2;
        cx[c_num] = x1 - fa * (x2 - x0);
        cy[c_num] = y1 - fa * (y2 - y0);
        cz[c_num++] = z1 - fa * (z2 - z0);
        cx[c_num] = x1 + fb * (x2 - x0);
        cy[c_num] = y1 + fb * (y2 - y0);
        cz[c_num] = z1 + fb * (z2 - z0);
    }
    for(int i = 0; i < tree1.size() - 1; i++){
        double t, x, y, z;
        x0 = tree1.at(i).x;
        y0 = tree1.at(i).y;
        z0 = tree1.at(i++).z;
        x1 = tree1.at(i).x;
        y1 = tree1.at(i).y;
        z1 = tree1.at(i--).z;
        for(int j = 0; j < num_points; j=j+40){
            t = 1.0 * j / (num_points - 1);
            int k1 = i * 2 + 1;
            int k2 = i * 2 + 2;
            x = x0 * pow((1 - t), 3) + 3 * cx[k1] * pow((1 - t), 2)*t + 3 * cx[k2] * pow(t, 2)*(1 - t) + x1 * pow(t, 3);
            y = y0 * pow((1 - t), 3) + 3 * cy[k1] * pow((1 - t), 2)*t + 3 * cy[k2] * pow(t, 2)*(1 - t) + y1 * pow(t, 3);
            z = z0 * pow((1 - t), 3) + 3 * cz[k1] * pow((1 - t), 2)*t + 3 * cz[k2] * pow(t, 2)*(1 - t) + z1 * pow(t, 3);
            worldPts.push_back(10*float(x)+110);
            worldPts.push_back(10*float(y)+40);
            worldPts.push_back(10*float(z)+100);
        }
    }
    
    return worldPts;
}


//Euclidean distance
double matric(double nx, double ny, double nz, double x2, double y2, double z2) {

    double dis2Obs2 = ((nx - x2) * (nx - x2)) + ((ny - y2) * (ny - y2)) + ((nz - z2) * (nz - z2));
    double d2O2 = sqrt(dis2Obs2);

    return d2O2;
}

//Collision check
bool checkCollision(double nx, double ny, double nz, double x2, double y2, double z2, double radius) {
    double distance = matric(nx, ny, nz, x2, y2, z2);
    return distance <= radius;
}

// csv file for the path
void writeCSV(std::vector<theNode> tree1, int j, int number) {
    double xvalue[j];
    double yvalue[j];
    double zvalue[j];
    int pvalue[j];



    //path points
    for (int k = 0; k < tree1.size(); ++k) {
        xvalue[k] = tree1.at(k).x;
        yvalue[k] = tree1.at(k).y;
        zvalue[k] = tree1.at(k).z;
        pvalue[k] = tree1.at(k).parent;
    }

    //create a csv file
    std::ofstream myFile("path" + to_string(number) + ".csv");

    for (int l = 0; l < tree1.size(); ++l) {
        myFile << xvalue[l] << ", " << yvalue[l] << ", " << zvalue[l];
        myFile << "\n";
    }
    myFile.close();
}


// The algorithm itself
vector<float> rrtAlgorithm(double xstart, double ystart, double zstart, double gx, double gy, double gz, double goalRadius, double ax1, double ay1, double az1, double obsRadius, double dmax, int iterations) {

    //** define the environment
    int xmin = 0;
    int ymin = 0;
    int zmin = 0;
    int xmax = 20;
    int ymax = 20;
    int zmax = 20;

    //a vector containing all the nodes with its parent values
    std::vector<theNode> tree;
    std::vector<theNode> obstacle;
    std::vector<theNode> goal;
    theNode node1 = *createNode(xstart, ystart, zstart, 0);
    tree.push_back(node1);

    //random coordinates generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> positionX(xmin, xmax);
    std::uniform_real_distribution<double> positionY(ymin, ymax);
    std::uniform_real_distribution<double> positionZ(zmin, zmax);
    gen.seed(time(NULL));

    int j = 0;
    //creating and adding new point to the tree
    while (true) {

        double xrand = positionX(gen);
        double yrand = positionY(gen);
        double zrand = positionZ(gen);

        //finding the minimum distance from the random node(distance vector)
        int parentindex = 0;
        theNode tempNearNode;

        // Bias
        if (j % 10 != 0) {
            parentindex = FindNearest(xrand, yrand, zrand, tree).first;
            tempNearNode = FindNearest(xrand, yrand, zrand, tree).second;
        }
        else {
            parentindex = FindNearest(gx, gy, gz, tree).first;
            tempNearNode = FindNearest(gx, gy, gz, tree).second;
        }

        //calculation for adding a point to certain distance to the direction of random point
        double x0 = tempNearNode.x;
        double y0 = tempNearNode.y;
        double z0 = tempNearNode.z;
        double x1 = xrand;
        double y1 = yrand;
        double z1 = zrand;

        //adding the point
        double nx;
        double ny;
        double nz;
        double d = matric(x1, y1, z1, x0, y0, z0);
        double t = dmax / d;
        if (d > dmax) {
            nx = ((1 - t) * x0) + t * x1;
            ny = ((1 - t) * y0) + t * y1;
            nz = ((1 - t) * z0) + t * z1;
        }
        else {
            nx = x1;
            ny = y1;
            nz = z1;
        }

        theNode newNode = *createNode(nx, ny, nz, parentindex);

        //Obstacle Check
        bool inCollision = checkCollision(nx, ny, nz, ax1, ay1, az1, obsRadius);
        if (!inCollision) {
            tree.push_back(newNode);
        }
        if (inCollision) {
            obstacle.push_back(newNode);
        }

        // Goal checking
        if (checkCollision(nx, ny, nz, gx, gy, gz, goalRadius)) {
            goal.push_back(newNode);
            theNode lastNode = *createNode(gx, gy, gz, parentindex - 1);
            tree.push_back(lastNode);
            break;      // for only one solution, comment out this command
        }

        if (j == iterations) {
            cout << "iteration limit reached" << endl;
            break;     //for more than one solution, comment out command
        }
        j++;
    }

    // printTree(obstacle, obstacle.size());
    // cout << j << "   "<< goal.size()<< endl ;

    vector <vector <theNode>> resltvec;
    //backtracking
    vector<float> worldPts;
    for (int i = 0; i < goal.size(); ++i) {
        vector<theNode> resultpoints;
        theNode resultNode = goal.at(i); // here there will be a goal node
        int position = resultNode.parent;

        while (position > 0) {
            theNode temp;
            temp.x = resultNode.x;
            temp.y = resultNode.y;
            temp.z = resultNode.z;
            resultpoints.push_back(temp);

            position = resultNode.parent;
            resultNode = tree.at(position);
            if (position == 0) {
                resultpoints.push_back(node1);
            }
        }
        // the resultpoints vector will be in reverse order
        std::reverse(resultpoints.begin(), resultpoints.end());
        resultpoints.push_back(*createNode(gx, gy, gz, 0));
        resltvec.push_back(resultpoints);
        // print the path
        // printTree(resultpoints, resultpoints.size());
        // writeCSV(resultpoints, resultpoints.size(), i);
    //         ShowTree(resultpoints, resultpoints.size());
        worldPts=bezier_points(resultpoints, 100, resultpoints.size());

    }
    
    
    return worldPts;
}



int try_alpha() {

    // first 3 values for start point
    // second 3 values for target point and then goal region value
    // next 3 values are for obstacle position and then its size in radius
    // the last two values are the step size and the numbers of iterations to perform respectively.

    
    
    return 0;

}
