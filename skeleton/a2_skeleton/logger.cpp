#include "logger.h"
#include <iostream>


namespace logger{

bool loadPoints(std::string fileName, std::vector<pfms::geometry_msgs::Point>& points){

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
          pfms::geometry_msgs::Point pt;
          lineStream >> pt.x;
          lineStream >> pt.y;
        //   lineStream >> pt.z;
          points.push_back(pt);
        }
        else {
            //std::cerr << "Closing " << __func__  << std::endl;
            file.close();
        }
    }
    return true;
}


bool savePoints(std::string fileName, std::vector<pfms::geometry_msgs::Point> points){

    std::ofstream file;
    file.open(fileName ,  std::ios::out | std::ios::trunc);

    if (!file.is_open()){
        std::cerr << "Can not open " << fileName  << std::endl;
        return false;
    }
    std::string line;
    // Read one line at a time into the variable line:
    for (auto point : points)
    {
        // file << point.y << "," << point.y << "," << point.z << std::endl;
        file << point.y << "," << point.y << std::endl;
    }
    file.close();
    return true;
}

}
