#ifndef DISPLAY_H
#define DISPLAY_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <random>   // Includes the random number generator

#include "car.h"

const unsigned int MAP_SIZE       =400;
const unsigned int MAP_CENTRE     =200;
const unsigned int RADIUS         =150;

class DisplayRace
{
public:
  DisplayRace(std::vector<CarInterface*> cars);
  void updateDisplay();

private:
  cv::Mat track_;
  std::vector<CarInterface*> cars_;

};

#endif // DISPLAY_H
