#include "logger.h"
#include <iostream>


namespace logger{

bool loadPoints(std::string fileName, std::vector<geometry_msgs::Point>& points){

    std::ifstream file(fileName , std::ios::in);

    points.clear();
    if (!file.is_open()){
        std::cerr << "Can not open " << fileName  << std::endl;
        return false;
    }
    std::string line;
    // Read one line at a time into the variable line:
    while (file.is_open()){
        if(std::getline(file, line))
        {
          std::stringstream  lineStream(line);
          geometry_msgs::Point pt;
          lineStream >> pt.x;
          lineStream >> pt.y;
          lineStream >> pt.z;
          points.push_back(pt);
        }
        else {
            //std::cerr << "Closing " << __func__  << std::endl;
            file.close();
        }
    }
    return true;
}


bool savePoints(std::string fileName, std::vector<geometry_msgs::Point> points){

    std::ofstream file(fileName , std::ios::out);

    points.clear();
    if (!file.is_open()){
        std::cerr << "Can not open " << fileName  << std::endl;
        return false;
    }
    // Save one line at a time into the file
    for (unsigned int i=0;i<points.size();i++){
        geometry_msgs::Point pt = points.at(i);
        file << pt.x;
        file << " " ;
        file << pt.y;
        file << " ";
        file << pt.z;

        if(i<points.size()-1){
            file << std::endl;
        }
    }
    file.close();
    return true;
}


std::string toString(std::vector<geometry_msgs::Point> points){
  std::stringstream ss;
  for (unsigned int idx=0;idx<points.size();idx++){
         ss << points.at(idx).x << " " <<
         points.at(idx).y << " " <<
         points.at(idx).z << std::endl;
    }
  return ss.str();
}

}


